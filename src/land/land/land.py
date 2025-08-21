#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from pymavlink import mavutil
import time
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data

# ArduPilot flight modes for reference
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
        self.declare_parameter('device', 'tcp:127.0.0.1:5762')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('pose_topic', '/target_pose')

        device = self.get_parameter('device').value
        baud = self.get_parameter('baud').value
        pose_topic = self.get_parameter('pose_topic').value

        # MAVLink connection
        self.get_logger().info(f"Connecting to {device} at {baud} baud")
        self.vehicle = mavutil.mavlink_connection(device, baud=baud)
        self.vehicle.wait_heartbeat()
        self.get_logger().info(f"Heartbeat received from system {self.vehicle.target_system}")

        # Publisher for landing status
        self.status_pub = self.create_publisher(Bool, '/landing_status', 10)

        # Optionally publish default status
        default_status = Bool()
        default_status.data = False
        self.status_pub.publish(default_status)

        # Subscriber to target pose
        self.subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {pose_topic}")

        # Timer to poll MAVLink messages
        self.timer = self.create_timer(0.1, self.poll_mavlink_messages)

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

        # self.get_logger().info(
        #     f"[STATUS] Mode: {flight_mode} | {'ARMED' if is_armed else 'DISARMED'} | {'LANDED' if is_landed else 'FLYING'}",
        #     throttle_duration_sec=2.0
        # )

        # Publish landing status depending on current mode and state
        msg = Bool()
        if flight_mode == "LAND" and is_flying:
            msg.data = True
            self.status_pub.publish(msg)
        elif flight_mode == "LAND" and not is_armed:
            msg.data = False
            self.status_pub.publish(msg)

    def pose_callback(self, msg):
        x_cam = msg.pose.position.x
        y_cam = msg.pose.position.y
        z_cam = msg.pose.position.z

        if z_cam <= 0:
            self.get_logger().warn("Invalid Z distance (<=0), ignoring message")
            return

        angle_x = math.atan2(x_cam, z_cam)
        angle_y = math.atan2(y_cam, z_cam)
        distance = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)

        try:
            landing_target_msg = self.vehicle.mav.landing_target_encode(
                0, 0, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                angle_x, angle_y, distance, 0.0, 0.0
            )
            self.vehicle.mav.send(landing_target_msg)
            # self.get_logger().info(
            #     f"Sent LANDING_TARGET: angles=({math.degrees(angle_x):.1f}°, {math.degrees(angle_y):.1f}°) dist={distance:.1f}m",
            #     throttle_duration_sec=0.1
            # )
        except Exception as e:
            self.get_logger().error(f"MAVLink send failed: {str(e)}")

    def poll_mavlink_messages(self):
        msg = self.vehicle.recv_match(blocking=False)
        if not msg:
            return

        if msg.get_type() == 'HEARTBEAT':
            self.handle_heartbeat(msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = LandingTargetBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
