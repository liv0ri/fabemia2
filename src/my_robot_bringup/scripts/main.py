#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
import random
import json
import subprocess
import time
from tsp_route_calculator import TSPRouteServer

HOUSES = [f'HOUSE_{i}' for i in range(1, 11)]

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.client = self.create_client(Trigger, 'calculate_tsp_route')

    def generate_targets(self):
        return random.sample(HOUSES, 3)

    def call_tsp(self, targets):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for TSP service...')

        request = TSPRouteServer.Request()
        request.targets = targets
        future = self.client.call_async(request)

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        data = json.loads(response.message)
        return data['targets_order']

    def go_to_house(self, start, target):
        self.get_logger().info(f"Navigating from {start} → {target}")

        subprocess.run([
            'ros2', 'run', 'my_robot_bringup', 'camera_follower',
            '--ros-args',
            '-p', f'target_house:={target}',
            '-p', f'start:={start}'
        ])

    def run(self):
        targets = self.generate_targets()
        self.get_logger().info(f"Generated targets: {targets}")

        ordered_targets = self.call_tsp(targets)
        self.get_logger().info(f"Optimal order: {ordered_targets}")

        current = 'PO'
        for house in ordered_targets:
            self.go_to_house(current, house)
            current = house
            time.sleep(2)

        # Return to PO
        self.go_to_house(current, 'PO')
        self.get_logger().info("Mission complete")

def main():
    rclpy.init()
    node = MissionController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
