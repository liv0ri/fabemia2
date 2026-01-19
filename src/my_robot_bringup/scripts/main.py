#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json
import subprocess
import time

HOUSES = [f'HOUSE_{i}' for i in range(1, 11)]

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.publisher = self.create_publisher(String, 'tsp_targets', 10)
        self.subscription = self.create_subscription(
            String,
            'tsp_route',
            self.route_callback,
            10
        )
        self.optimized_targets = None
        self.current = 'PO' 

        # Publish targets after 1 second
        self.create_timer(1.0, self.publish_targets_once)
        self.targets_sent = False

    def generate_targets(self):
        return random.sample(HOUSES, 3)

    def publish_targets_once(self):
        if not self.targets_sent:
            targets = self.generate_targets()
            msg = String()
            msg.data = json.dumps(targets)
            self.publisher.publish(msg)
            self.get_logger().info(f"Published targets: {targets}")
            self.targets_sent = True

    def route_callback(self, msg):
        if self.optimized_targets is None:
            self.optimized_targets = json.loads(msg.data)
            self.get_logger().info(f"Received optimized route: {self.optimized_targets}")
            self.start_mission()  # Trigger the mission automatically

    def go_to_house(self, start, target):
        self.get_logger().info(f"Navigating from {start} → {target}")
        subprocess.run([
            'ros2', 'run', 'my_robot_bringup', 'camera_follower',
            '--ros-args',
            '-p', f'target_house:={target}',
            '-p', f'start:={start}'
        ])

    def start_mission(self):
        for house in self.optimized_targets:
            self.go_to_house(self.current, house)
            self.current = house
            time.sleep(2)
        self.go_to_house(self.current, 'PO')
        self.get_logger().info("Mission complete")

def main():
    rclpy.init()
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
