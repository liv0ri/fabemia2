#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

import numpy as np
import cv2
import math
import json
from enum import Enum
from config import directions


# robots states
class Mode(Enum):
    FOLLOW_LINE = 1
    VERIFY_HOUSE = 2
    STOP = 3


class CameraFollower(Node):
    def __init__(self):
        super().__init__('camera_house_follower')

        # ===================== NAVIGATION =====================
        self.mode = Mode.FOLLOW_LINE
        self.navigation_active = False

        self.turn_plan = []
        self.turn_index = 0
        self.turn_active = False
        self.need_clear_intersection = False
        self.all_turns_complete = False

        # ===================== ODOM =====================
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0
        self.odom_ready = False

        # ===================== LINE FOLLOWING (ERROR TURNS) =====================
        self.line_found = False
        self.line_error = 0.0
        self.prev_line_error = 0.0

        self.kp_line = 0.8
        self.kd_line = 0.5
        self.max_error_angular = 0.25

        # ===================== ACTUAL TURNS =====================
        self.kp_turn = 1.8
        self.max_turn_angular = 0.5

        # ===================== SIDE CAMERAS =====================
        self.left_line = False
        self.right_line = False

        # ===================== HOUSE DETECTION =====================
        self.house_visible = False
        self.house_reached = False
        self.house_seen_frames = 0
        self.stop_ratio = 0.95

        # ===================== HOUSE COLOURS =====================
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

        self.house_colours = {}
        for name, rgb in colors.items():
            rgb_np = np.uint8([[list(rgb)]])
            hsv = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2HSV)[0][0]
            h, _, _ = hsv
            self.house_colours[name] = (
                (max(h - 5, 0), 100, 80),
                (min(h + 5, 179), 255, 255)
            )

        # ===================== ROS =====================
        self.cmd = Twist()

        self.front_sub = self.create_subscription(Image, '/front_camera/image_raw', self.front_callback, 10)
        self.bm_sub = self.create_subscription(Image, '/bm_camera/image_raw', self.bm_callback, 10)
        self.bl_sub = self.create_subscription(Image, '/bl_camera/image_raw', self.bl_callback, 10)
        self.br_sub = self.create_subscription(Image, '/br_camera/image_raw', self.br_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.nav_sub = self.create_subscription(String, 'navigate_to_house', self.nav_callback, qos)

        self.done_pub = self.create_publisher(String, 'navigation_done', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    # ===================== HELPERS =====================
    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def angle_error(self, target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def detect_black(self, hsv):
        return cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 60]))

    # ===================== ERROR TURN (LINE FOLLOW) =====================
    def calculate_error_turn(self, speed):
        d = self.line_error - self.prev_line_error
        angular = -(self.kp_line * self.line_error + self.kd_line * d)
        angular = max(min(angular, self.max_error_angular), -self.max_error_angular)
        angular *= speed / 0.22
        self.prev_line_error = self.line_error
        return speed, angular

    # ===================== ACTUAL TURN =====================
    def start_actual_turn(self, right, half=False):
        self.turn_active = True
        self.start_yaw = self.normalize_angle(self.current_yaw)
        delta = -math.pi / 2 if right else math.pi / 2
        if half:
            delta *= 2
        self.target_yaw = self.normalize_angle(self.start_yaw + delta)
        self.get_logger().info(f"START TURN {self.turn_index+1}/{len(self.turn_plan)}")

    def run_actual_turn(self):
        error = self.angle_error(self.target_yaw, self.current_yaw)
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = max(min(self.kp_turn * error, self.max_turn_angular),
                                 -self.max_turn_angular)

        if abs(error) < 0.02:
            self.turn_active = False
            self.turn_index += 1
            self.need_clear_intersection = True
            self.prev_line_error = 0.0

            if self.turn_index >= len(self.turn_plan):
                self.all_turns_complete = True
                self.get_logger().info("ALL TURNS COMPLETE")

    # ===================== CALLBACKS =====================
    def bm_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        roi = hsv[int(h * 0.6):h, :]
        mask = self.detect_black(roi)
        M = cv2.moments(mask)

        if M["m00"] > 800:
            cx = int(M["m10"] / M["m00"])
            self.line_error = (cx - w / 2) / (w / 2)
            self.line_found = True
        else:
            self.line_found = False

    def bl_callback(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        self.left_line = np.sum(self.detect_black(hsv) > 0) > 500

    def br_callback(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        self.right_line = np.sum(self.detect_black(hsv) > 0) > 500

    def front_callback(self, msg):
        if not self.all_turns_complete:
            return

        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
        center = mask[:, w//2-80:w//2+80]

        self.house_visible = np.sum(mask > 0) > 1200
        self.house_reached = (np.sum(center > 0) / center.size) > self.stop_ratio

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        self.odom_ready = True

    # ===================== NAV =====================
    def nav_callback(self, msg):
        data = json.loads(msg.data)
        self.start = data['start']
        self.target = data['target']

        self.turn_plan = directions[self.start][self.target]
        self.turn_index = 0
        self.turn_active = False
        self.need_clear_intersection = False
        self.all_turns_complete = False

        self.lower, self.upper = self.house_colours[self.target]

        self.house_visible = False
        self.house_reached = False
        self.house_seen_frames = 0
        self.mode = Mode.FOLLOW_LINE

        self.navigation_active = True
        self.get_logger().info(f"Navigate {self.start} → {self.target}")

    # ===================== CONTROL LOOP =====================
    def control_loop(self):
        if not self.navigation_active or not self.odom_ready:
            self.publisher.publish(Twist())
            return

        if self.turn_active:
            self.run_actual_turn()
            self.publisher.publish(self.cmd)
            return

        intersection = self.line_found and (self.left_line or self.right_line)

        if intersection and not self.need_clear_intersection and not self.all_turns_complete:
            self.need_clear_intersection = True
            self.start_actual_turn(self.turn_plan[self.turn_index])
            return

        if not intersection:
            self.need_clear_intersection = False

        if self.mode == Mode.FOLLOW_LINE:
            if self.line_found:
                self.cmd.linear.x, self.cmd.angular.z = self.calculate_error_turn(0.22)
            else:
                self.cmd.linear.x = 0.05
                self.cmd.angular.z = 0.0

            if self.all_turns_complete and self.house_visible:
                self.house_seen_frames += 1
                if self.house_seen_frames > 2:
                    self.mode = Mode.VERIFY_HOUSE
            else:
                self.house_seen_frames = 0

        elif self.mode == Mode.VERIFY_HOUSE:
            if self.line_found:
                self.cmd.linear.x, self.cmd.angular.z = self.calculate_error_turn(0.08)

            if self.house_reached:
                self.mode = Mode.STOP
                self.get_logger().info("HOUSE REACHED")

        elif self.mode == Mode.STOP:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)

            msg = String()
            msg.data = json.dumps({"reached": self.target})
            self.done_pub.publish(msg)

            self.navigation_active = False
            return

        self.publisher.publish(self.cmd)

def main():
    rclpy.init()
    node = CameraFollower()
    node.get_logger().info("Waiting for simulation to initialize...")

    node.get_logger().info("Starting control loop...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Stop the robot
    stop_cmd = Twist()
    node.publisher.publish(stop_cmd)
    node.get_logger().info('Shutting down - Robot stopped')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
