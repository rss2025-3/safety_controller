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
        self.declare_parameter("safety_topic", "default")
        self.declare_parameter("stopping_time", 0.0)


        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter('safety_topic').get_parameter_value().string_value
        self.STOPPING_TIME = self.get_parameter('stopping_time').get_parameter_value().double_value

        self.drive_publisher_ = self.create_publisher(AckermannDriveStamped, self.SAFETY_TOPIC, 10)

        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            self.DRIVE_TOPIC,
            self.acker_callback,
            10)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.listener_callback,
            10)
        
        self.velocity = 1

    def acker_callback(self, msg):
        self.velocity = msg.drive.speed

    def listener_callback(self, laser_scan):
        
        scan_data = laser_scan.ranges

        forward_dist = np.mean(scan_data[45:55])
        time_to_collision = forward_dist / self.velocity
        stop = time_to_collision < self.STOPPING_TIME
        
        
        if stop:
            # Do something to stop
            current_time = self.get_clock().now()
            drive_stamped = AckermannDriveStamped()
            drive_stamped.header.frame_id = "drive_frame_id"
            drive_stamped.header.stamp = current_time.to_msg()
            drive_stamped.drive.speed = 0.0
            drive_stamped.drive.steering_angle = 0.0

            self.get_logger().info(f'STOPPING!!! {time_to_collision=}')

            # TODO: uncomment this to send actual stop commands
            # self.drive_publisher_.publish(drive_stamped)

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
