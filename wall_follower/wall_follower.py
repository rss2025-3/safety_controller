#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", 1)  # 1 for left, -1 for right (inverted logic)
        self.declare_parameter("velocity", 1.3)
        self.declare_parameter("desired_distance", 1.0)

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
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

        # PID constants
        # self.kp = 0.3
        # self.ki = 0.0  
        # self.kd = 0.1

        #self.kp = 0.3
        self.kp = 1.1
        self.ki = 0.0  
        self.kd = 0.1

        
        # PID error tracking
        self.prev_error = 0.0
        self.integral = 0.0        
        self.prev_time = self.get_clock().now()

        # Visualization publishers and tools
        self.vis_publishers = [
            self.create_publisher(Marker, f'/wall_follower/viz_{i}', 10)
            for i in range(3)
        ]
        self.viz_tools = VisualizationTools()
        

    def fit_line(self, angles, ranges):
        """Fit a line (y = m*x + c) to polar coordinates converted into Cartesian."""
        # Convert polar to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        # Perform linear regression
        A = np.vstack([x, np.ones(len(x))]).T
        m, c = np.linalg.lstsq(A, y, rcond=None)[0]
        
        # Generate points for visualization
        x_viz = np.array([np.min(x), np.max(x)])
        y_viz = m * x_viz + c
        
        return m, c, x_viz, y_viz


    def listener_callback(self, laser_scan):
        #self.SIDE = -1

        #self.get_logger().info(f"On {self.SIDE}")

        # Create an array of angles corresponding to the LIDAR scan measurements
        angles = np.arange(
            laser_scan.angle_min,
            laser_scan.angle_max + laser_scan.angle_increment,
            laser_scan.angle_increment
        )

        measured_distance = None

        # ----- Regression on the Wall Section -----
        # With the inverted logic:
        # For left wall (SIDE == 1): use scan angles from 0 to 4π/6 and a fallback angle of π/3.
        # For right wall (SIDE == -1): use scan angles from -4π/6 to 0 and a fallback angle of -π/3.
        if self.SIDE == 1:  # left wall
            #section_min, section_max = (np.pi/6, 4*np.pi/6)
            section_min, section_max = (-np.pi/6, 4*np.pi/6)
            fallback_angle = np.pi/2
        else:              # right wall (SIDE == -1)
            #section_min, section_max = (-4*np.pi/6, -np.pi/6)
            section_min, section_max = (-4*np.pi/6, np.pi/6)
            fallback_angle = -np.pi/2

        indices = np.where((angles >= section_min) & (angles <= section_max))[0]

        if len(indices) > 2:
            section_angles = angles[indices]
            section_ranges = np.array(laser_scan.ranges)[indices]
            valid = np.isfinite(section_ranges)
            if np.sum(valid) > 2:
                # Fit a line to the valid LIDAR points in the section
                m, c, x_viz, y_viz = self.fit_line(
                    section_angles[valid],
                    section_ranges[valid]
                )

                # Assume car_half_width is known (e.g., 0.05 meters)
                #car_half_width = 0.7
                car_half_width = 0.3

                # Determine lookahead ray angle based on side:
                #lookahead_offset = np.pi / 3  # 60 degrees in radians
                lookahead_offset = np.pi / 4  # reduced offset for earlier turning
                #lookahead_offset = 4*np.pi / 12  # reduced offset for earlier turning

                if self.SIDE == 1:
                    # Left wall: start at (0, car_half_width), ray angle = π/2 - 60° = π/6
                    base_angle = np.pi / 2
                    lookahead_ray_angle = base_angle - lookahead_offset  # π/2 - π/3 = π/6
                    # Solve: car_half_width + t*sin(lookahead_ray_angle) = m*(t*cos(lookahead_ray_angle)) + c
                    denominator = np.sin(lookahead_ray_angle) - m * np.cos(lookahead_ray_angle)
                    if np.abs(denominator) > 1e-6:
                        t = (c - car_half_width) / denominator
                    else:
                        t = np.inf
                else:
                    # Right wall: start at (0, -car_half_width), ray angle = -π/2 + 60° = -π/6
                    base_angle = -np.pi / 2
                    lookahead_ray_angle = base_angle + lookahead_offset  # -π/2 + π/3 = -π/6
                    denominator = np.sin(lookahead_ray_angle) - m * np.cos(lookahead_ray_angle)
                    if np.abs(denominator) > 1e-6:
                        t = (c + car_half_width) / denominator
                    else:
                        t = np.inf

                # Use t as the projected (lookahead) distance if it is positive and finite
                if np.isfinite(t) and t > 0:
                    measured_distance = t
                else:
                    # Fallback: use the perpendicular distance calculation
                    if self.SIDE == 1:
                        measured_distance = abs(car_half_width - c) / np.sqrt(m**2 + 1)
                    else:
                        measured_distance = abs(car_half_width + c) / np.sqrt(m**2 + 1)

                # # Assume car_half_width is known (e.g., 0.5 meters)
                # car_half_width = 0.05  

                # # Calculate the distance from the appropriate side of the car to the fitted line.
                # # For left wall (SIDE == 1), use the point (0, car_half_width)
                # # For right wall (SIDE == -1), use the point (0, -car_half_width)
                # if self.SIDE == 1:
                #     measured_distance = abs(car_half_width - c) / np.sqrt(m**2 + 1)
                # else:
                #     measured_distance = abs(car_half_width + c) / np.sqrt(m**2 + 1)

                # side_str = "left" if self.SIDE == 1 else "right"
                # self.get_logger().info(f"Measured distance from {side_str} side: {measured_distance}")

                # Visualize the fitted line on the corresponding publisher
                VisualizationTools.plot_line(
                    x_viz, y_viz,
                    self.vis_publishers[0],
                    color=(1.0, 0.0, 0.0),
                    frame=laser_scan.header.frame_id
                )
        
        # ----- Fallback to Single Ray Measurement -----
        if measured_distance is None or measured_distance == 0.0:
            index = int((fallback_angle - laser_scan.angle_min) / laser_scan.angle_increment)
            measured_distance = laser_scan.ranges[index]
            if not np.isfinite(measured_distance):
                measured_distance = 0.0

        # ----- PID Controller for Steering -----
        desired_distance = self.DESIRED_DISTANCE
        desired_velocity = self.VELOCITY

        error = desired_distance - measured_distance

        #self.get_logger().info(f"Recorded error is: {error}")

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        steering_correction = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        # To steer away from the wall:
        # For left wall (SIDE == 1): steer right (negative angle).
        # For right wall (SIDE == -1): steer left (positive angle).
        drive = AckermannDrive()
        drive.steering_angle = -self.SIDE * steering_correction  

        drive.speed = desired_velocity
        
        drive_stamped = AckermannDriveStamped()
        drive_stamped.header.frame_id = "drive_frame_id"
        drive_stamped.header.stamp = current_time.to_msg()
        drive_stamped.drive = drive
        
        self.drive_publisher_.publish(drive_stamped)

        # Update PID state
        self.prev_error = error
        self.prev_time = current_time

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
