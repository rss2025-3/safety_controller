#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from safety_controller.visualization_tools import VisualizationTools

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", 1)  # 1 for left, -1 for right (inverted logic)
        self.declare_parameter("velocity", 1.3)
        self.declare_parameter("desired_distance", 1.0)

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        self.add_on_set_parameters_callback(self.parameters_callback)
  
        drive_topic = self.get_parameter("drive_topic").value
        lidar_topic = self.get_parameter("scan_topic").value

        self.drive_publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            lidar_topic,
            self.listener_callback,
            10)

        # Visualization publishers and tools
        self.vis_publishers = [
            self.create_publisher(Marker, f'/wall_follower/viz_{i}', 10)
            for i in range(3)
        ]
        self.viz_tools = VisualizationTools()


    def listener_callback(self, laser_scan):
        # Create an array of angles corresponding to the LIDAR scan measurements
        angles = np.arange(
            laser_scan.angle_min,
            laser_scan.angle_max + laser_scan.angle_increment,
            laser_scan.angle_increment
        )

        current_time = self.get_clock().now()
        drive = AckermannDrive()
        drive.steering_angle = 0

        drive.speed = self.VELOCITY
        
        drive_stamped = AckermannDriveStamped()
        drive_stamped.header.frame_id = "drive_frame_id"
        drive_stamped.header.stamp = current_time.to_msg()
        drive_stamped.drive = drive
        
        self.drive_publisher_.publish(drive_stamped)

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
