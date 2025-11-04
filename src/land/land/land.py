#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import Image, CameraInfo, Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from transforms3d.euler import euler2quat
import math
import time
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
from pymavlink import mavutil
from rclpy.qos import qos_profile_sensor_data

# ----------------------------------------------------------------------
# ArduPilot flight mode table
# ----------------------------------------------------------------------
ARDUPILOT_FLIGHT_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT",
    14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
    19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD",
    23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL"
}

# ----------------------------------------------------------------------
# Combined node
# ----------------------------------------------------------------------
class LandingTargetBridge(Node):
    def __init__(self):
        super().__init__('landing_target_bridge')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('baud', 57600)
        self.declare_parameter('pose_topic', '/target_pose')
        self.baud = self.get_parameter('baud').value
        pose_topic = self.get_parameter('pose_topic').value

        # ------------------------------------------------------------------
        # MAVLink handling
        # ------------------------------------------------------------------
        self.vehicle = None
        self.connected = False
        self.stop_thread = False
        self.device = None

        # ------------------------------------------------------------------
        # Publishers (original bridge)
        # ------------------------------------------------------------------
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
        self.compass_use_pub = self.create_publisher(Bool, '/compass_use', 10)

        # ------------------------------------------------------------------
        # Subscriptions (original + optical flow)
        # ------------------------------------------------------------------
        self.subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {pose_topic}")

        # ---- Optical Flow -------------------------------------------------
        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_time = None

        self.focal_length_x = None      # fx (pixels)
        self.focal_length_y = None      # fy (pixels)
        self.cx = None
        self.cy = None

        self.latest_distance = 0.0      # ground distance (m) from rangefinder
        self.distance_lock = threading.Lock()

        self.create_subscription(
            Image, '/camera', self.image_callback, 10)
        self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10)

        # ------------------------------------------------------------------
        # TF broadcaster
        # ------------------------------------------------------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # ------------------------------------------------------------------
        # State variables
        # ------------------------------------------------------------------
        self.home_lat = None
        self.home_lon = None
        self.last_orientation = None
        self.last_vfr = None
        self.last_mode = None
        self.last_mode_time = time.time()
        self.compass_info = {}

        # ------------------------------------------------------------------
        # Threads / timers
        # ------------------------------------------------------------------
        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()
        self.timer = self.create_timer(1.0, self.periodic_tasks)

        self.get_logger().info("LandingTargetBridge + OpticalFlow started")

    # ==================================================================
    # MAVLink connection & stream requests
    # ==================================================================
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

                # Request full param list (for compass, fence, etc.)
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
                msg_id, interval_us, 0, 0, 0, 0, 0
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
        self.set_message_interval('RC_CHANNELS', 2)

    # ==================================================================
    # MAVLink listener (receives all messages, including rangefinder)
    # ==================================================================
    def mavlink_listener(self):
        self.get_logger().info("Starting MAVLink listener thread.")
        while not self.stop_thread and rclpy.ok():
            if not self.connected:
                self.connect_to_mavlink()
                continue
            try:
                msg = self.vehicle.recv_match(
                    type=[
                        'HEARTBEAT', 'SCALED_RANGEFINDER', 'DISTANCE_SENSOR',
                        'RC_CHANNELS', 'FENCE_STATUS', 'BATTERY_STATUS',
                        'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3',
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
                elif msg_type == 'BATTERY_STATUS':
                    self.handle_battery_status(msg)
                elif msg_type == 'STATUSTEXT':
                    self.handle_statustext(msg)
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

    # ==================================================================
    # Message handlers (original bridge)
    # ==================================================================
    def handle_heartbeat(self, msg):
        system_status = msg.system_status
        base_mode = msg.base_mode
        custom_mode = msg.custom_mode
        autopilot = msg.autopilot
        mav_type = msg.type

        if autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA and \
           mav_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_TYPE_GENERIC]:
            mode_str = ARDUPILOT_FLIGHT_MODES.get(custom_mode, f"UNKNOWN({custom_mode})")
            if mode_str != self.last_mode or (time.time() - self.last_mode_time) > 2:
                self.flight_mode_pub.publish(String(data=mode_str))
                self.last_mode = mode_str
                self.last_mode_time = time.time()
            is_armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.armed_status_pub.publish(Bool(data=is_armed))
            self.land_armed_pub.publish(Bool(data=(mode_str == "LAND" and is_armed)))
            self.land_disarmed_pub.publish(Bool(data=(mode_str == "LAND" and not is_armed)))

    def handle_rangefinder(self, msg):
        """Updates internal distance (used by optical flow) and publishes."""
        distance_m = 0.0
        if msg.get_type() == 'SCALED_RANGEFINDER':
            distance_m = msg.distance / 100.0
        elif msg.get_type() == 'DISTANCE_SENSOR':
            distance_m = msg.current_distance / 100.0

        with self.distance_lock:
            self.latest_distance = distance_m

        self.current_altitude = distance_m
        self.rangefinder_pub.publish(Float32(data=distance_m))

    def handle_rc_channels(self, msg):
        if hasattr(msg, 'chan8_raw'):
            self.rcin_ch8_pub.publish(Bool(data=msg.chan8_raw > 1500))
        if hasattr(msg, 'chan6_raw'):
            self.rcin_ch6_pub.publish(Bool(data=msg.chan6_raw > 1500))

    def request_fence_enable(self):
        if self.connected and self.vehicle:
            try:
                self.vehicle.mav.param_request_read_send(
                    self.vehicle.target_system,
                    self.vehicle.target_component,
                    b'FENCE_ENABLE', -1
                )
            except Exception as e:
                self.get_logger().warn(f"Failed to request FENCE_ENABLE: {e}")

    def handle_battery_status(self, msg):
        voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] != 0xFFFF else 0.0
        current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
        self.battery_voltage_pub.publish(Float32(data=voltage))
        self.battery_current_pub.publish(Float32(data=current))

    def handle_statustext(self, msg):
        text = msg.text.lower()
        if "external compass" in text:
            self.external_compass_pub.publish(String(data=text))

    def handle_gps(self, msg):
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0

        if self.home_lat is None:
            self.home_lat = lat
            self.home_lon = lon

        # ---- local ENU ----
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
        pose.pose.orientation = self.last_orientation if self.last_orientation else \
                                self.create_quaternion_msg(1.0, 0.0, 0.0, 0.0)
        self.pose_pub.publish(pose)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = getattr(self, 'current_altitude', 0.0)
        t.transform.rotation = self.last_orientation if self.last_orientation else \
                               self.create_quaternion_msg(1.0, 0.0, 0.0, 0.0)
        self.tf_broadcaster.sendTransform(t)

    def create_quaternion_msg(self, w, x=0.0, y=0.0, z=0.0):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.w, q.x, q.y, q.z = w, x, y, z
        return q

    def handle_attitude(self, msg):
        roll, pitch, yaw = -msg.roll, -msg.pitch, -msg.yaw + math.pi/2
        q = quaternion_from_euler(roll, pitch, yaw)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
        imu.angular_velocity.x = msg.rollspeed
        imu.angular_velocity.y = msg.pitchspeed
        imu.angular_velocity.z = msg.yawspeed

        if self.last_vfr:
            imu.linear_acceleration.x = self.last_vfr['airspeed']
            imu.linear_acceleration.z = -self.last_vfr['climb']

        self.imu_pub.publish(imu)

        self.last_orientation = self.create_quaternion_msg(q[3], q[0], q[1], q[2])

    def handle_vfr_hud(self, msg):
        self.last_vfr = {
            'airspeed': msg.airspeed,
            'groundspeed': msg.groundspeed,
            'heading': msg.heading,
            'throttle': msg.throttle,
            'alt': msg.alt,
            'climb': msg.climb
        }
        self.heading_pub.publish(Float32(data=float(msg.heading)))

    def handle_param_value(self, msg):
        param_id = msg.param_id.rstrip('\x00')
        param_value = msg.param_value

        if param_id.startswith("COMPASS_"):
            self.compass_info[param_id] = param_value

        required = [
            'COMPASS_DEV_ID', 'COMPASS_DEV_ID2', 'COMPASS_DEV_ID3',
            'COMPASS_USE', 'COMPASS_USE2', 'COMPASS_USE3',
            'COMPASS_PRIMARY'
        ]
        if all(p in self.compass_info for p in required):
            self.identify_active_compass()

        if param_id == 'FENCE_ENABLE':
            self.fence_enabled_pub.publish(Bool(data=int(param_value) == 1))
        if param_id == 'COMPASS_EXTERNAL':
            self.compass_use_pub.publish(Bool(data=bool(int(param_value))))

    def identify_active_compass(self):
        EXTERNAL_UAVCAN = 97539.0
        INTERNAL_SPI = 590114.0
        dev_ids = [int(self.compass_info.get(f'COMPASS_DEV_ID{i}', 0)) for i in (1, 2, 3)]
        primary_idx = int(self.compass_info.get('COMPASS_PRIMARY', 0))
        primary_id = dev_ids[primary_idx]

        if primary_id == EXTERNAL_UAVCAN:
            name = "External (UAVCAN)"
        elif primary_id == INTERNAL_SPI:
            name = "Internal (SPI)"
        else:
            name = f"Unknown (DEV_ID={primary_id})"

        self.compass_name_pub.publish(String(data=name))

    def handle_gps_raw(self, msg):
        self.gps_satellites_pub.publish(Int32(data=msg.satellites_visible))
        fix_desc = {
            0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix",
            4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"
        }.get(msg.fix_type, f"Unknown ({msg.fix_type})")
        self.gps_status_pub.publish(String(data=fix_desc))

    # ==================================================================
    # Landing target pose -> MAVLink
    # ==================================================================
    def pose_callback(self, msg: PoseStamped):
        if not self.connected:
            self.get_logger().warn("Not connected to MAVLink. Skipping pose.")
            return

        x_cam = msg.pose.position.x
        y_cam = msg.pose.position.y
        z_cam = msg.pose.position.z
        if z_cam <= 0:
            self.get_logger().warn("Invalid Z (<=0). Skipping.")
            return

        angle_x = math.atan2(x_cam, z_cam)
        angle_y = math.atan2(y_cam, z_cam)
        distance = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)

        try:
            mav_msg = self.vehicle.mav.landing_target_encode(
                0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
                angle_x, angle_y, distance, 0.0, 0.0
            )
            self.vehicle.mav.send(mav_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to send LANDING_TARGET: {e}")

    # ==================================================================
    # Optical Flow (integrated)
    # ==================================================================
    def camera_info_callback(self, msg: CameraInfo):
        if self.focal_length_x is None:
            self.focal_length_x = msg.k[0]      # fx
            self.focal_length_y = msg.k[4]      # fy
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f"Camera intrinsics: fx={self.focal_length_x}, fy={self.focal_length_y}, "
                f"cx={self.cx}, cy={self.cy}"
            )

    def image_callback(self, msg: Image):
        if not self.connected:
            return

        # Convert to mono8
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        if self.prev_gray is None or self.focal_length_x is None:
            self.prev_gray = gray
            self.prev_time = time.time()
            return

        # ---- distance (from rangefinder) --------------------------------
        with self.distance_lock:
            dist = self.latest_distance
        if dist <= 0.0:
            self.prev_gray = gray
            self.prev_time = time.time()
            return

        # ---- time delta -------------------------------------------------
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0.0:
            dt = 0.033          # fallback ~30 Hz
        self.prev_time = now

        # ---- Farneback optical flow --------------------------------------
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None,
            pyr_scale=0.5, levels=3, winsize=21,
            iterations=5, poly_n=7, poly_sigma=1.5, flags=0
        )

        flow_x_px = np.mean(flow[..., 0])
        flow_y_px = np.mean(flow[..., 1])

        # ---- pixel → rad -------------------------------------------------
        flow_x_rad = flow_x_px / self.focal_length_x
        flow_y_rad = flow_y_px / self.focal_length_y

        # ---- angular rates (rad/s) ---------------------------------------
        flow_rate_x = flow_x_rad / dt
        flow_rate_y = flow_y_rad / dt

        # ---- compensated linear flow (m/s) -------------------------------
        flow_comp_m_x = np.tan(flow_x_rad) * dist / dt
        flow_comp_m_y = np.tan(flow_y_rad) * dist / dt

        # ---- quality heuristic -------------------------------------------
        flow_mag_px = np.sqrt(flow_x_px**2 + flow_y_px**2)
        quality = int(np.clip(flow_mag_px * 50, 0, 255))

        # ---- send MAVLink OPTICAL_FLOW -----------------------------------
        flow_x_mrad_s = int(flow_rate_x)   # millirad/s
        flow_y_mrad_s = int(flow_rate_y)

        try:
            self.vehicle.mav.optical_flow_send(
                time_usec=int(now * 1e6),
                sensor_id=0,
                flow_x=flow_x_mrad_s,
                flow_y=flow_y_mrad_s,
                flow_comp_m_x=float(flow_comp_m_x),
                flow_comp_m_y=float(flow_comp_m_y),
                quality=quality,
                ground_distance=float(dist),
                flow_rate_x=float(flow_rate_x),
                flow_rate_y=float(flow_rate_y)
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to send OPTICAL_FLOW: {e}")

        self.get_logger().info(
            f"OF: px=({flow_x_px:.2f},{flow_y_px:.2f}) rad=({flow_x_rad:.6f},{flow_y_rad:.6f}) "
            f"mrad/s=({flow_x_mrad_s},{flow_y_mrad_s}) comp=({flow_comp_m_x:.3f},{flow_comp_m_y:.3f})m/s "
            f"dist={dist:.2f}m qual={quality}"
        )

        # ---- update previous frame ---------------------------------------
        self.prev_gray = gray

    # ==================================================================
    # Periodic tasks (currently empty)
    # ==================================================================
    def periodic_tasks(self):
        pass

    # ==================================================================
    # Clean shutdown
    # ==================================================================
    def destroy_node(self):
        self.stop_thread = True
        if self.mavlink_thread.is_alive():
            self.mavlink_thread.join(timeout=5)
        super().destroy_node()


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LandingTargetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
