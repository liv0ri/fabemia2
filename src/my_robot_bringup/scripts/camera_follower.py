#!/usr/bin/env python3

# Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
# control robot velocity through this
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from enum import Enum
from config import directions
import math

# robots states
class Mode(Enum):
    FOLLOW_LINE = 1
    VERIFY_HOUSE = 2
    STOP = 3

class CameraFollower(Node):
    # call it with the house its finding
    def __init__(self, target_house="HOUSE_2", start="PO"):
        super().__init__('camera_house_follower')
        # set a target house
        self.TARGET_HOUSE = target_house
        # The start position
        self.start = start
        self.mode = Mode.FOLLOW_LINE

        # Direction plan
        self.turn_plan = directions[start][self.TARGET_HOUSE]
        self.get_logger().info(f"Turning plan: {self.turn_plan}")
        self.turn_index = 0
        self.doing_turn = False
        self.current_turn_right = True
        # searching for the house
        self.search_house = False
        self.first_turn_done = False

        # Odometry
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0

        # Subscriptions
        self.front_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.front_callback,
            10
        )

        # Bottom-middle - main line following
        self.bm_sub = self.create_subscription(
            Image,
            '/bm_camera/image_raw',
            self.bm_callback,
            10
        )

        # Bottom-left - intersection / left line
        self.bl_sub = self.create_subscription(
            Image,
            '/bl_camera/image_raw',
            self.bl_callback,
            10
        )

        # Bottom-right - intersection / right line
        self.br_sub = self.create_subscription(
            Image,
            '/br_camera/image_raw',
            self.br_callback,
            10
        )

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishes velocity commands to the robot
        # linear.x - forward/backward
        # angular.z - left/right
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Loop every 0.05 seconds
        self.create_timer(0.05, self.control_loop)

        self.line_found = False
        # offset from center
        self.line_error = 0.0
        # Used when line is lost
        # track last line seen
        self.last_line_seen = False  
        # track last error 
        self.last_line_error = 0.0     
        # detects lines on the left or right camera
        self.left_line = False
        self.right_line = False

        self.house_visible = False
        self.house_reached = False
        # Count the number of times the house was seen
        # This is done to switch to house detection mode
        self.house_seen_frames = 0

        # Stop distance proxy image-based - when house fills 25% of center
        # fine tuned based on experiement with house 2
        self.stop_ratio = 0.95

        # Exact RGB colors from Gazebo diffuse values
        colors = {
            "HOUSE_1": (97, 63, 0),
            "HOUSE_2": (253, 255, 0),
            "HOUSE_3": (255, 161, 0),
            "HOUSE_4": (252, 149, 209),
            "HOUSE_5": (255, 16, 0),
            "HOUSE_6": (0, 251, 255),
            "HOUSE_7": (255, 0, 229),
            "HOUSE_8": (120, 255, 255),
            "HOUSE_9": (0, 44, 255),
            "HOUSE_10": (146, 220, 255),
            "PO": (255, 255, 255),
            "C0": (0, 255, 45),
            "C1": (0, 255, 45),
            "C2": (0, 255, 45),
        }

        # Convert exact RGB to HSV using OpenCV
        self.house_colours = {}
        for name, rgb in colors.items():
            rgb_np = np.uint8([[list(rgb)]])
            hsv = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2HSV)[0][0]

            # widen HSV range to tolerate lighting
            h, s, v = hsv
            # self.get_logger().info("Value of h in hsv", h, s, v, "in name", name)
            # widen HSV range to tolerate lighting and distinguish similar hues
            lower = (max(h - 5, 0), 100, 80)
            upper = (min(h + 5, 179), 255, 255)

            self.house_colours[name] = (lower, upper)

        # prevent errors
        if self.TARGET_HOUSE not in self.house_colours:
            raise RuntimeError("Unknown house selected")

        self.lower, self.upper = self.house_colours[self.TARGET_HOUSE]

        self.get_logger().info("Camera House Follower Started")
        self.get_logger().info(f"Target house: {self.TARGET_HOUSE}")

    # Helper function used throughout this class
    def detect_black(self, hsv):
        # returns a mask of black pixels in the imag
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        return cv2.inRange(hsv, lower_black, upper_black)

    def bm_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        roi = hsv[int(h * 0.6):h, :]
        # Check for black in this camera
        mask = self.detect_black(roi)

        # calulates center of black line in ROI Refion of Interest
        M = cv2.moments(mask)
        if M["m00"] > 800:
            # computer error from center to steer
            cx = int(M["m10"] / M["m00"])
            self.line_found = True
            self.line_error = cx - (w // 2)
            self.last_line_seen = True
            self.last_line_error = self.line_error
        else:
            self.line_found = False
    
    # check if enough pixels is found in any side camera
    # help to detect which side to move
    def bl_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Same as right
        mask = self.detect_black(hsv)
        self.left_line = np.sum(mask > 0) > 500

    def br_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = self.detect_black(hsv)
        self.right_line = np.sum(mask > 0) > 500

    def front_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        center = mask[:, w//2 - 80:w//2 + 80]
        # Is the house detectable
        self.house_visible = np.sum(mask > 0) > 1200
        self.house_reached = (np.sum(center > 0) / center.size) > self.stop_ratio

    def odom_callback(self, msg):
        # get yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def start_turn(self, turn_right):
        self.doing_turn = True
        self.start_yaw = self.current_yaw
        delta = -math.pi/2 if turn_right else math.pi/2
        self.target_yaw = (self.start_yaw + delta + math.pi) % (2*math.pi) - math.pi

    def control_loop(self):
        cmd = Twist()

        self.get_logger().debug(
            f"Loop | first_turn_done={self.first_turn_done} "
            f"doing_turn={self.doing_turn} turn_index={self.turn_index}"
        )
        # Force first turn immediately at start
        if not self.first_turn_done:
            self.start_turn(self.turn_plan[0])
            self.first_turn_done = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 if self.current_turn_right else -0.8
            self.cmd_pub.publish(cmd)
            self.get_logger().info("FORCED FIRST TURN COMMAND SENT")
            return

        if self.mode == Mode.FOLLOW_LINE:
            if self.doing_turn:
                cmd.linear.x = 0.0
                
                # Calculate the shortest path to target yaw
                error = self.target_yaw - self.current_yaw
                # Normalize error to [-pi, pi]
                error = (error + math.pi) % (2 * math.pi) - math.pi
                
                # Use Proportional control: speed depends on how much error is left
                # Gain of 2.0 is a good start; min speed of 0.2 ensures it doesn't stall
                kp_rot = 2.0
                rot_speed = kp_rot * error
                
                # Cap the speed so it doesn't spin wildly
                max_rot_speed = 2.0
                if rot_speed > max_rot_speed: rot_speed = max_rot_speed
                if rot_speed < -max_rot_speed: rot_speed = -max_rot_speed
                
                cmd.angular.z = rot_speed

                # Check if turn completed threshold of ~3 degrees
                if abs(error) < 0.05:
                    self.doing_turn = False
                    self.turn_index += 1
                    if self.turn_index >= len(self.turn_plan):
                        self.search_house = True
                    self.get_logger().info(f"TURN COMPLETE, index={self.turn_index}")
                    cmd.angular.z = 0.0
                
                self.cmd_pub.publish(cmd)
                return

            if self.left_line and self.right_line and not self.search_house:
                if self.turn_index < len(self.turn_plan):
                    self.start_turn(self.turn_plan[self.turn_index])
                    self.cmd_pub.publish(cmd)
                    return

            if self.line_found:
                cmd.linear.x = 0.22
                cmd.angular.z = -self.line_error * 0.003
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = -np.sign(self.last_line_error) * 0.8

            if self.left_line and self.right_line and not self.search_house:
                if self.turn_index < len(self.turn_plan):
                    self.start_turn(self.turn_plan[self.turn_index])
                    self.cmd_pub.publish(cmd)
                    return

            if self.search_house and self.house_visible:
                self.house_seen_frames += 1
                if self.house_seen_frames > 8:
                    self.mode = Mode.VERIFY_HOUSE
            else:
                self.house_seen_frames = 0

        elif self.mode == Mode.VERIFY_HOUSE:
            if self.line_found:
                cmd.linear.x = 0.15
                cmd.angular.z = -self.line_error * 0.003

            if self.house_reached:
                self.mode = Mode.STOP

        elif self.mode == Mode.STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

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
