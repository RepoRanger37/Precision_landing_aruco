#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool, String, Int32
import math
from pymavlink import mavutil
import time
import threading
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from transforms3d.euler import euler2quat
from tf_transformations import euler_from_quaternion, quaternion_from_euler



ARDUPILOT_FLIGHT_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT",
    14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
    19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD",
    23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL"
}


class LandingTargetBridge(Node):
    def __init__(self):
        super().__init__('landing_target_bridge')

        self.declare_parameter('baud', 57600)
        self.declare_parameter('pose_topic', '/target_pose')

        self.baud = self.get_parameter('baud').value
        pose_topic = self.get_parameter('pose_topic').value

        self.vehicle = None
        self.connected = False
        self.stop_thread = False
        self.device = None

        # Publishers
        self.flight_mode_pub = self.create_publisher(String, '/flight_mode', 10)
        self.armed_status_pub = self.create_publisher(Bool, '/armed_status', 10)
        self.land_armed_pub = self.create_publisher(Bool, '/land_armed_status', 10)
        self.land_disarmed_pub = self.create_publisher(Bool, '/land_disarmed_status', 10)
        self.rcin_ch8_pub = self.create_publisher(Bool, '/rcin_channel_8', 10)
        self.rangefinder_pub = self.create_publisher(Float32, '/rf_distance', 10)
        self.rcin_ch6_pub = self.create_publisher(Bool, '/rcin_channel_6', 10)
        self.fence_enabled_pub = self.create_publisher(Bool, '/fence_enabled', 10)
        self.battery_voltage_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.battery_current_pub = self.create_publisher(Float32, '/battery_current', 10)
        self.gyro_ok_pub = self.create_publisher(Bool, '/gyro_ok', 10)
        self.mag_field_pub = self.create_publisher(Float32, '/magfield', 10)
        self.external_compass_pub = self.create_publisher(String, '/external_compass', 10)
        self.gps_status_pub = self.create_publisher(String, '/gps_status', 10)
        self.gps_satellites_pub = self.create_publisher(Int32, '/gps_satellites', 10)
        self.compass_name_pub = self.create_publisher(String, '/compass', 10)
        self.imu_pub = self.create_publisher(Imu, '/drone_imu', 10)
        self.gps_fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.heading_pub = self.create_publisher(Float32, '/drone/heading', 10)

        self.subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {pose_topic}")

        self.tf_broadcaster = TransformBroadcaster(self)


        self.home_lat = None
        self.home_lon = None
        self.last_orientation = None
        self.last_vfr = None
        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()

        self.timer = self.create_timer(1.0, self.periodic_tasks)

        self.compass_info = {}

    def connect_to_mavlink(self):
        port = "/dev/ttyUSB0"
        while not self.connected and not self.stop_thread:
            self.get_logger().info(f"Trying to connect to {port} at {self.baud} baud...")
            try:
                self.vehicle = mavutil.mavlink_connection(port, baud=self.baud)
                self.vehicle.wait_heartbeat(timeout=5)
                self.get_logger().info(f"Connected via {port}. Heartbeat received.")
                self.device = port
                self.connected = True
                self.request_data_streams()
                self.request_rc_channels()
                self.last_mode = None
                self.last_mode_time = time.time()
                self.fence_check_timer = self.create_timer(10.0, self.request_fence_enable)
                self.compass_use_pub = self.create_publisher(Bool, '/compass_use', 10)

                # Request full parameter list
                self.vehicle.mav.param_request_list_send(
                    self.vehicle.target_system,
                    self.vehicle.target_component
                )
                return
            except Exception as e:
                self.get_logger().warn(f"Failed to connect on {port}: {e}")
                time.sleep(2)

    def set_message_interval(self, msg_name, rate_hz):
        try:
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{msg_name}")
            interval_us = int(1_000_000 / rate_hz)
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0, 0, 0, 0, 0
            )
            self.get_logger().info(f"Requested {msg_name} at {rate_hz} Hz")
        except Exception as e:
            self.get_logger().warn(f"Failed to request {msg_name}: {e}")

    def request_data_streams(self):
        messages = {
            'SCALED_RANGEFINDER': 10,
            'DISTANCE_SENSOR': 10,
            'RC_CHANNELS': 2,
            'SCALED_IMU': 10,
            'SCALED_IMU2': 10,
            'SCALED_IMU3': 10,
            'BATTERY_STATUS': 1,
            'GPS_RAW_INT': 5,
            'EKF_STATUS_REPORT': 1,
            'STATUSTEXT': 1,
            'FENCE_STATUS': 1,
            'ATTITUDE': 10,
            'VFR_HUD': 5,
            'GLOBAL_POSITION_INT': 5
        }
        for name, hz in messages.items():
            self.set_message_interval(name, hz)

    def request_rc_channels(self):
        try:
            self.set_message_interval('RC_CHANNELS', 2)
        except Exception as e:
            self.get_logger().error(f"Failed to request RC_CHANNELS: {e}")

    def mavlink_listener(self):
        self.get_logger().info("Starting MAVLink listener thread.")
        while not self.stop_thread and rclpy.ok():
            if not self.connected:
                self.connect_to_mavlink()
                continue
            try:
                msg = self.vehicle.recv_match(
                    type=[
                        'HEARTBEAT', 'SCALED_RANGEFINDER', 'DISTANCE_SENSOR', 'RC_CHANNELS',
                        'FENCE_STATUS', 'BATTERY_STATUS', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3',
                        'STATUSTEXT', 'EKF_STATUS_REPORT', 'GLOBAL_POSITION_INT',
                        'PARAM_VALUE', 'GPS_RAW_INT', 'ATTITUDE', 'VFR_HUD'
                    ],
                    blocking=True,
                    timeout=2.0
                )
                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type == 'HEARTBEAT':
                    self.handle_heartbeat(msg)
                elif msg_type in ['SCALED_RANGEFINDER', 'DISTANCE_SENSOR']:
                    self.handle_rangefinder(msg)
                elif msg_type == 'RC_CHANNELS':
                    self.handle_rc_channels(msg)
                # elif msg_type == 'FENCE_STATUS':
                #     self.handle_fence_status(msg)
                elif msg_type == 'BATTERY_STATUS':
                    self.handle_battery_status(msg)
                # elif msg_type in ['SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3']:
                #     self.handle_imu_values(msg)
                elif msg_type == 'STATUSTEXT':
                    # handle STATUSTEXT messages
                    self.handle_statustext(msg)
                #     self.handle_ekf_status(msg)
                elif msg_type == 'GLOBAL_POSITION_INT':
                    self.handle_gps(msg)
                elif msg_type == 'PARAM_VALUE':
                    self.handle_param_value(msg)
                elif msg_type == 'GPS_RAW_INT':
                    self.handle_gps_raw(msg)
                elif msg_type == 'ATTITUDE':
                    self.handle_attitude(msg)
                elif msg_type == 'VFR_HUD':
                    self.handle_vfr_hud(msg)

            except Exception as e:
                self.get_logger().warn(f"MAVLink communication lost: {e}. Reconnecting...")
                self.connected = False
                time.sleep(2)

    def handle_heartbeat(self, msg):
        system_status = msg.system_status
        base_mode = msg.base_mode
        custom_mode = msg.custom_mode
        autopilot = msg.autopilot
        mav_type = msg.type

    # Only process modes from the actual flight controller (e.g., copter, plane)
        if autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA and mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_TYPE_GENERIC]:

            mode_str = ARDUPILOT_FLIGHT_MODES.get(custom_mode, f"UNKNOWN({custom_mode})")

        # Prevent flickering: only publish if mode actually changed or enough time passed
            if mode_str != self.last_mode or (time.time() - self.last_mode_time) > 2:
                self.flight_mode_pub.publish(String(data=mode_str))
                self.last_mode = mode_str
                self.last_mode_time = time.time()

            is_armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.armed_status_pub.publish(Bool(data=is_armed))
            self.land_armed_pub.publish(Bool(data=(mode_str == "LAND" and is_armed)))
            self.land_disarmed_pub.publish(Bool(data=(mode_str == "LAND" and not is_armed)))

    def handle_rangefinder(self, msg):
        distance_m = 0.0
        if msg.get_type() == 'SCALED_RANGEFINDER':
            distance_m = msg.distance / 100.0
        elif msg.get_type() == 'DISTANCE_SENSOR':
            distance_m = msg.current_distance / 100.0

        self.current_altitude = distance_m
        self.rangefinder_pub.publish(Float32(data=distance_m))

    def handle_rc_channels(self, msg):
        # Channel 8
        if hasattr(msg, 'chan8_raw'):
            is_ch8_active = msg.chan8_raw > 1500
            self.rcin_ch8_pub.publish(Bool(data=is_ch8_active))
        else:
            self.get_logger().warn("chan8_raw not found in RC_CHANNELS message")

    # Channel 6
        if hasattr(msg, 'chan6_raw'):
            is_ch6_active = msg.chan6_raw > 1500
            self.rcin_ch6_pub.publish(Bool(data=is_ch6_active))
        else:
            self.get_logger().warn("chan6_raw not found in RC_CHANNELS message")

    def request_fence_enable(self):
        if self.connected and self.vehicle:
            try:
                self.vehicle.mav.param_request_read_send(
                    self.vehicle.target_system,
                    self.vehicle.target_component,
                    b'FENCE_ENABLE',
                    -1
                )
                self.get_logger().debug("Requested FENCE_ENABLE param")
            except Exception as e:
                self.get_logger().warn(f"Failed to request FENCE_ENABLE: {e}")


    def handle_battery_status(self, msg):
        voltages = msg.voltages
        voltage = voltages[0] / 1000.0 if voltages[0] != 0xFFFF else 0.0
        current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0

        self.battery_voltage_pub.publish(Float32(data=voltage))
        self.battery_current_pub.publish(Float32(data=current))

    def identify_active_compass(self):
        # Hardcoded DEV_IDs
        EXTERNAL_UAVCAN_DEV_ID = 97539.0
        INTERNAL_SPI_DEV_ID = 590114.0

        dev_ids = [
            int(self.compass_info.get('COMPASS_DEV_ID', 0)),
            int(self.compass_info.get('COMPASS_DEV_ID2', 0)),
            int(self.compass_info.get('COMPASS_DEV_ID3', 0))
        ]

        uses = [
            int(self.compass_info.get('COMPASS_USE', 0)),
            int(self.compass_info.get('COMPASS_USE2', 0)),
            int(self.compass_info.get('COMPASS_USE3', 0))
        ]

        primary_idx = int(self.compass_info.get('COMPASS_PRIMARY', 0))
        primary_dev_id = dev_ids[primary_idx]

        if primary_dev_id == EXTERNAL_UAVCAN_DEV_ID:
            compass_name = "External (UAVCAN)"
        elif primary_dev_id == INTERNAL_SPI_DEV_ID:
            compass_name = "Internal (SPI)"
        else:
            compass_name = f"Unknown Compass (DEV_ID={primary_dev_id})"

        self.get_logger().info(f"Active compass: {compass_name}")
        self.compass_name_pub.publish(String(data=compass_name))

    def handle_statustext(self, msg):
        text = msg.text.lower()
        if "external compass" in text:
            self.external_compass_pub.publish(String(data=text))


    def handle_gps(self, msg):
        lat = msg.lat / 1e7 
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        # self.get_logger().info(f"GPS Location: Lat={lat}, Lon={lon}, Alt={alt}m")

        if self.home_lat is None:
            self.home_lat = lat
            self.home_lon = lon

        # Convert to local ENU
        R = 6378137.0
        dlat = math.radians(lat - self.home_lat)
        dlon = math.radians(lon - self.home_lon)
        x = R * dlon * math.cos(math.radians((lat + self.home_lat) / 2))
        y = R * dlat
        z = alt

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # If we already have orientation from ATTITUDE, attach it
        if self.last_orientation:
            pose.pose.orientation = self.last_orientation
        else:
            pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = getattr(self, 'current_altitude', 0.0)
        if self.last_orientation:
            t.transform.rotation = self.last_orientation
        else:
            t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


    def handle_attitude(self, msg):
        roll, pitch, yaw = -msg.roll, -msg.pitch, -msg.yaw + math.pi/2


        q = quaternion_from_euler(roll, pitch, yaw)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]

        imu.angular_velocity.x = msg.rollspeed
        imu.angular_velocity.y = msg.pitchspeed
        imu.angular_velocity.z = msg.yawspeed

        # Add last known linear acceleration (approx from VFR_HUD)
        if self.last_vfr:
            imu.linear_acceleration.x = self.last_vfr['airspeed']
            imu.linear_acceleration.z = -self.last_vfr['climb']

        self.imu_pub.publish(imu)

        # Save orientation for use in pose
        from geometry_msgs.msg import Quaternion
        q_msg = Quaternion()
        q_msg.x, q_msg.y, q_msg.z, q_msg.w = q[0], q[1], q[2], q[3]
        self.last_orientation = q_msg

    def handle_vfr_hud(self, msg):
        self.last_vfr = {
            'airspeed': msg.airspeed,
            'groundspeed': msg.groundspeed,
            'heading': msg.heading,
            'throttle': msg.throttle,
            'alt': msg.alt,
            'climb': msg.climb
        }
        heading = msg.heading
        heading_msg = Float32()
        heading_msg.data = float(heading)
        self.heading_pub.publish(heading_msg)

    def handle_param_value(self, msg):
        try:
            param_id = msg.param_id.rstrip('\x00')
        except Exception as e:
            self.get_logger().warn(f"Failed to decode param_id: {e}")
            return
        
        param_value = msg.param_value
        if param_id.startswith("COMPASS_"):
            self.compass_info[param_id] = param_value
            self.get_logger().info(f"Compass param: {param_id} = {param_value}")

        # Wait until all required compass params are available
        required_params = [
            'COMPASS_DEV_ID', 'COMPASS_DEV_ID2', 'COMPASS_DEV_ID3',
            'COMPASS_USE', 'COMPASS_USE2', 'COMPASS_USE3',
            'COMPASS_PRIMARY'
        ]
        if all(p in self.compass_info for p in required_params):
            self.display_compass_usage()
            self.identify_active_compass()

        if param_id == 'FENCE_ENABLE':
            fence_enabled = int(param_value) == 1
            # self.get_logger().info(f"[PARAM] FENCE_ENABLE = {fence_enabled}")
            self.fence_enabled_pub.publish(Bool(data=fence_enabled))

        if param_id == 'COMPASS_EXTERNAL':
            is_external = bool(int(param_value))
            self.get_logger().info(f"[PARAM] COMPASS_EXTERNAL = {is_external}")
            self.compass_use_pub.publish(Bool(data=is_external))

    def handle_gps_raw(self, msg):
        fix_type = msg.fix_type
        satellites = msg.satellites_visible
        self.gps_satellites_pub.publish(Int32(data=satellites))
        fix_description = {
            0: "No GPS",
            1: "No Fix",
            2: "2D Fix",
            3: "3D Fix",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed"
        }.get(fix_type, f"Unknown ({fix_type})")
        self.gps_status_pub.publish(String(data=fix_description))

        # Populate NavSatFix
        gps_msg = NavSatFix()

        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
        if fix_type >= 2:
            gps_msg.status.status = NavSatStatus.STATUS_FIX
        if fix_type >= 4:
            gps_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        if fix_type >= 5:
            gps_msg.status.status = NavSatStatus.STATUS_GBAS_FIX


    def pose_callback(self, msg: PoseStamped):
        if not self.connected:
            self.get_logger().warn("Not connected to MAVLink. Skipping pose.")
            return
        
        x_cam = msg.pose.position.x
        y_cam = msg.pose.position.y
        z_cam = msg.pose.position.z

        if z_cam <= 0:
            self.get_logger().warn("Invalid Z (<= 0). Skipping.")
            return
        
        angle_x = math.atan2(x_cam, z_cam)
        angle_y = math.atan2(y_cam, z_cam)
        distance = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)

        try:
            msg = self.vehicle.mav.landing_target_encode(
                0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
                angle_x, angle_y, distance, 0.0, 0.0
            )
            self.vehicle.mav.send(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to send LANDING_TARGET: {e}")
        
        # self.get_logger().info(f"Received landing target pose: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def periodic_tasks(self):
        pass

    def destroy_node(self):
        self.stop_thread = True
        if self.mavlink_thread.is_alive():
            self.mavlink_thread.join(timeout=5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LandingTargetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LandingTargetBridge node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

