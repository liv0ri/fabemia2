#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class CameraFollower(Node):
    def __init__(self):
        super().__init__('camera_follower')
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to odometry to estimate distance
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.last_image_time = self.get_clock().now()
        self.object_detected = False
        self.distance_estimate = 999.0  # Start with large distance
        
        # Detection threshold - objects darker than this
        self.dark_threshold = 120  # Increased from 100
        
        # Minimum distance to object (in meters)
        self.min_distance = 0.3  # 30 cm - safer distance
        
        self.get_logger().info('Camera Follower Started')
        self.get_logger().info(f'Looking for objects darker than {self.dark_threshold}...')
        self.get_logger().info(f'Will stop at {self.min_distance}m from object')
    
    def odom_callback(self, msg):
        """Track odometry for distance estimation"""
        # We'll use image size as a proxy for distance
        pass
    
    def estimate_distance_from_image(self, dark_pixels_count):
        """Estimate distance based on how much of image is dark"""
        # More dark pixels = closer to object
        # This is a rough approximation
        total_pixels = 640 * 480
        dark_ratio = dark_pixels_count / total_pixels
        
        if dark_ratio > 0.3:  # Object fills >30% of view
            return 0.2  # Very close
        elif dark_ratio > 0.15:  # Object fills >15% of view
            return 0.4  # Close
        elif dark_ratio > 0.05:  # Object fills >5% of view
            return 0.8  # Medium distance
        else:
            return 2.0  # Far away
    
    def image_callback(self, msg):
        """Process camera image and follow dark objects"""
        # Convert image message to numpy array
        height = msg.height
        width = msg.width
        
        # Convert bytes to numpy array and reshape
        img_array = np.array(msg.data, dtype=np.uint8).reshape(height, width, 3)
        
        # Convert to grayscale (average of RGB)
        gray = np.mean(img_array, axis=2)
        
        # Find dark pixels
        dark_mask = gray < self.dark_threshold
        dark_pixels_count = np.sum(dark_mask)
        
        # Get center column for forward detection
        center_col = width // 2
        stripe_width = 100  # Wider stripe for better detection
        center_stripe = gray[:, center_col-stripe_width:center_col+stripe_width]
        center_dark_pixels = np.sum(center_stripe < self.dark_threshold)
        center_dark_ratio = center_dark_pixels / center_stripe.size
        
        # Get left and right regions
        left_region = gray[:, :width//3]
        right_region = gray[:, 2*width//3:]
        
        left_dark = np.sum(left_region < self.dark_threshold)
        right_dark = np.sum(right_region < self.dark_threshold)
        
        # Estimate distance
        self.distance_estimate = self.estimate_distance_from_image(dark_pixels_count)
        
        # Create velocity command
        cmd = Twist()
        
        # Check if we see a dark object ahead
        if center_dark_ratio > 0.1:  # At least 10% of center is dark
            self.object_detected = True
            
            # Check distance
            if self.distance_estimate > self.min_distance:
                # Move forward
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'Object ahead! Distance: ~{self.distance_estimate:.2f}m - Moving forward'
                )
            else:
                # Too close - stop
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'Reached object! Distance: ~{self.distance_estimate:.2f}m - STOPPED'
                )
        else:
            # No object in center - turn towards darkest region
            self.object_detected = False
            
            if left_dark > right_dark and left_dark > 100:
                cmd.angular.z = 0.4  # Turn left
                self.get_logger().info(f'Searching... Turning LEFT (dark pixels: {left_dark})')
            elif right_dark > 100:
                cmd.angular.z = -0.4  # Turn right  
                self.get_logger().info(f'Searching... Turning RIGHT (dark pixels: {right_dark})')
            else:
                # Nothing detected anywhere - slow spin
                cmd.angular.z = 0.3
                self.get_logger().info('No objects detected - Slow search rotation')
        
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = CameraFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Stop the robot
    stop_cmd = Twist()
    node.cmd_pub.publish(stop_cmd)
    node.get_logger().info('Shutting down - Robot stopped')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()