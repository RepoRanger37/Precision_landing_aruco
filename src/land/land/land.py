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

        # ROS2 parameters
        self.declare_parameter('baud', 115200)
        self.declare_parameter('pose_topic', '/target_pose')

        self.baud = self.get_parameter('baud').value
        pose_topic = self.get_parameter('pose_topic').value

        # Connection state
        self.vehicle = None
        self.connected = False
        self.stop_thread = False
        self.device = None  # Will be set after connection

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/landing_status', 10)
        self.rangefinder_pub = self.create_publisher(Float32, '/rf_distance', 10)
        self.status_pub.publish(Bool(data=False))  # default status

        # Subscriber to vision pose
        self.subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {pose_topic}")

        # Start MAVLink thread
        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()

        # Optional periodic tasks
        self.timer = self.create_timer(1.0, self.periodic_tasks)

    def connect_to_mavlink(self):
        ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
        try_limits = {
            "/dev/ttyACM0": 5,  # initial tries for ACM0
            "/dev/ttyACM1": 3   # initial tries for ACM1
        }
        fallback_try_limit = 3  # subsequent tries after initial attempts

        current_port_index = 0
        port_attempts = {"/dev/ttyACM0": 0, "/dev/ttyACM1": 0}
        max_tries_exceeded = False

        while not self.connected and not self.stop_thread:
            current_port = ports[current_port_index]

            # Determine how many tries to allow this round
            if port_attempts[current_port] < try_limits[current_port]:
                max_attempts = try_limits[current_port]
            else:
                max_attempts = fallback_try_limit

            for attempt in range(max_attempts):
                if self.stop_thread or self.connected:
                    return

                self.get_logger().info(
                    f"[{current_port}] Attempt {attempt+1}/{max_attempts} at {self.baud} baud..."
                )
                try:
                    self.vehicle = mavutil.mavlink_connection(current_port, baud=self.baud)
                    self.vehicle.wait_heartbeat(timeout=5)
                    self.get_logger().info(f"Connected via {current_port}. Heartbeat received.")
                    self.device = current_port
                    self.connected = True
                    self.request_data_streams()
                    return
                except Exception as e:
                    self.get_logger().warn(f"Failed on {current_port}: {e}")
                    time.sleep(2)

            # Mark this port as attempted
            port_attempts[current_port] += max_attempts
            current_port_index = (current_port_index + 1) % len(ports)

    def request_data_streams(self):
        try:
            self.vehicle.mav.request_data_stream_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                2,  # rate in Hz
                1   # start
            )
            self.get_logger().info("Requested data streams from FCU")
        except Exception as e:
            self.get_logger().error(f"Failed to request data streams: {str(e)}")

    def mavlink_listener(self):
        self.get_logger().info("Starting MAVLink listener thread.")
        while not self.stop_thread and rclpy.ok():
            if not self.connected:
                self.connect_to_mavlink()
                continue

            try:
                msg = self.vehicle.recv_match(
                    type=['HEARTBEAT', 'SCALED_RANGEFINDER', 'DISTANCE_SENSOR'],
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

            except Exception as e:
                self.get_logger().warn(f"MAVLink communication lost: {e}. Reconnecting...")
                self.connected = False
                time.sleep(2)

    def handle_heartbeat(self, msg):
        custom_mode = msg.custom_mode
        base_mode = msg.base_mode
        system_status = msg.system_status

        flight_mode = ARDUPILOT_FLIGHT_MODES.get(custom_mode, f"UNKNOWN({custom_mode})")
        is_armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        is_landed = system_status in [
            mavutil.mavlink.MAV_STATE_STANDBY,
            mavutil.mavlink.MAV_STATE_BOOT,
            mavutil.mavlink.MAV_STATE_POWEROFF,
        ]
        is_flying = not is_landed

        status_msg = Bool()
        status_msg.data = (flight_mode == "LAND") and is_flying
        self.status_pub.publish(status_msg)

    def handle_rangefinder(self, msg):
        distance_m = 0.0
        if msg.get_type() == 'SCALED_RANGEFINDER':
            distance_m = msg.distance / 100.0
        elif msg.get_type() == 'DISTANCE_SENSOR':
            distance_m = msg.current_distance / 100.0
        self.rangefinder_pub.publish(Float32(data=distance_m))

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

