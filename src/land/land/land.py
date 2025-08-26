#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool
import math
from pymavlink import mavutil
import time
import threading
from rclpy.qos import qos_profile_sensor_data

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

        self.declare_parameter('baud', 115200)
        self.declare_parameter('pose_topic', '/target_pose')

        self.baud = self.get_parameter('baud').value
        pose_topic = self.get_parameter('pose_topic').value

        self.vehicle = None
        self.connected = False
        self.stop_thread = False
        self.device = None

        self.land_armed_pub = self.create_publisher(Bool, '/land_armed_status', 10)
        self.land_disarmed_pub = self.create_publisher(Bool, '/land_disarmed_status', 10)
        self.rcin_ch8_pub = self.create_publisher(Bool, '/rcin_channel_8', 10)
        self.rangefinder_pub = self.create_publisher(Float32, '/rf_distance', 10)

        self.subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {pose_topic}")

        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()

        self.timer = self.create_timer(1.0, self.periodic_tasks)

    def connect_to_mavlink(self):
        ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
        try_limits = {"/dev/ttyACM0": 5, "/dev/ttyACM1": 3}
        fallback_try_limit = 3

        current_port_index = 0
        port_attempts = {"/dev/ttyACM0": 0, "/dev/ttyACM1": 0}

        while not self.connected and not self.stop_thread:
            current_port = ports[current_port_index]
            max_attempts = try_limits[current_port] if port_attempts[current_port] < try_limits[current_port] else fallback_try_limit

            for attempt in range(max_attempts):
                if self.stop_thread or self.connected:
                    return

                self.get_logger().info(f"[{current_port}] Attempt {attempt + 1}/{max_attempts} at {self.baud} baud...")
                try:
                    self.vehicle = mavutil.mavlink_connection(current_port, baud=self.baud)
                    self.vehicle.wait_heartbeat(timeout=5)
                    self.get_logger().info(f"Connected via {current_port}. Heartbeat received.")
                    self.device = current_port
                    self.connected = True
                    self.request_data_streams()
                    self.request_rc_channels()
                    return
                except Exception as e:
                    self.get_logger().warn(f"Failed on {current_port}: {e}")
                    time.sleep(2)

            port_attempts[current_port] += max_attempts
            current_port_index = (current_port_index + 1) % len(ports)

    def request_data_streams(self):
        try:
            self.vehicle.mav.request_data_stream_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                2,
                1
            )
            self.get_logger().info("Requested data streams from FCU")
        except Exception as e:
            self.get_logger().error(f"Failed to request data streams: {str(e)}")

    def request_rc_channels(self):
        try:
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
                int(1e6 / 2), 0, 0, 0, 0, 0
            )
            self.get_logger().info("Requested RC_CHANNELS message at 2Hz")
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
                    type=['HEARTBEAT', 'SCALED_RANGEFINDER', 'DISTANCE_SENSOR', 'RC_CHANNELS'],
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

            except Exception as e:
                self.get_logger().warn(f"MAVLink communication lost: {e}. Reconnecting...")
                self.connected = False
                time.sleep(2)

    def handle_heartbeat(self, msg):
        custom_mode = msg.custom_mode
        base_mode = msg.base_mode

        flight_mode = ARDUPILOT_FLIGHT_MODES.get(custom_mode, f"UNKNOWN({custom_mode})")
        is_armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

        # self.get_logger().info(f"Flight Mode: {flight_mode} | Armed: {is_armed}")

        land_armed_msg = Bool()
        land_disarmed_msg = Bool()
        land_armed_msg.data = (flight_mode == "LAND") and is_armed
        land_disarmed_msg.data = (flight_mode == "LAND") and not is_armed

        self.land_armed_pub.publish(land_armed_msg)
        self.land_disarmed_pub.publish(land_disarmed_msg)

    def handle_rangefinder(self, msg):
        distance_m = 0.0
        if msg.get_type() == 'SCALED_RANGEFINDER':
            distance_m = msg.distance / 100.0
        elif msg.get_type() == 'DISTANCE_SENSOR':
            distance_m = msg.current_distance / 100.0

        self.rangefinder_pub.publish(Float32(data=distance_m))

    def handle_rc_channels(self, msg):
        ch8_pwm = getattr(msg, 'chan8_raw', None)
        if ch8_pwm is not None:
            is_active = ch8_pwm > 1500
            self.rcin_ch8_pub.publish(Bool(data=is_active))
            # self.get_logger().info(f"CH8 PWM: {ch8_pwm} µs -> {'ON' if is_active else 'OFF'}")
        else:
            self.get_logger().warn("chan8_raw not found in RC_CHANNELS message")

    def pose_callback(self, msg):
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
        distance = math.sqrt(x_cam ** 2 + y_cam ** 2 + z_cam ** 2)

        try:
            msg = self.vehicle.mav.landing_target_encode(
                0, 0, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                angle_x, angle_y, distance, 0.0, 0.0
            )
            self.vehicle.mav.send(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to send LANDING_TARGET: {e}")

    def periodic_tasks(self):
        pass

    def destroy_node(self):
        self.stop_thread = True
        time.sleep(1)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    bridge = LandingTargetBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("Keyboard interrupt. Shutting down.")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

