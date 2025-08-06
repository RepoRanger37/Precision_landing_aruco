#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from pymavlink import mavutil
import time
from rclpy.qos import qos_profile_sensor_data

class LandingTargetBridge(Node):
    def __init__(self):
        super().__init__('landing_target_bridge')
        
        # ROS parameters
        self.declare_parameter('device', '/dev/ttyACM1')
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
        
        # Pose subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info(f"Subscribed to {pose_topic}")

    def pose_callback(self, msg):
        # Extract camera-frame coordinates (OpenCV convention)
        x_cam = msg.pose.position.x  # Right
        y_cam = msg.pose.position.y  # Down
        z_cam = msg.pose.position.z  # Forward
        
        # Validate Z distance
        if z_cam <= 0:
            self.get_logger().warn("Invalid Z distance (<=0), ignoring message")
            return
        
        # Calculate angles and distance
        angle_x = math.atan2(x_cam, z_cam)
        angle_y = math.atan2(y_cam, z_cam)
        distance = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        
        # Send MAVLink message
        try:
            msg = self.vehicle.mav.landing_target_encode(
                0,  # time_boot_us (not used)
                0,  # target num
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                angle_x,
                angle_y,
                distance,
                0.0, 0.0  # size_x/y (optional)
            )
            self.vehicle.mav.send(msg)
            self.get_logger().info(
                f"Sent LANDING_TARGET: angles=({math.degrees(angle_x):.1f}°, {math.degrees(angle_y):.1f}°) dist={distance:.1f}m",
                throttle_duration_sec=0.1  # Throttle to 1Hz
            )
        except Exception as e:
            self.get_logger().error(f"MAVLink send failed: {str(e)}")

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
