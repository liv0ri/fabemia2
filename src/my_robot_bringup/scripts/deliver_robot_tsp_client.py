#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
import json
import random
import subprocess
import time

HOUSES = [f'HOUSE_{i}' for i in range(1, 11)]

class DeliverobotTSPClient(Node):
    def __init__(self):
        super().__init__('deliverobot_tsp_client')

        # Create TSP service client
        self.client = self.create_client(Trigger, 'calculate_tsp_route')

        # Generate 3 random targets
        self.targets = random.sample(HOUSES, 3)
        self.get_logger().info(f"Generated targets: {self.targets}")

        # Publish targets for logging or monitoring (optional)
        self.publisher = self.create_publisher(String, 'tsp_targets', 10)
        self.publish_targets()

        # Start calling TSP service
        self.call_tsp_service_until_ready()

    def publish_targets(self):
        msg = String()
        msg.data = json.dumps(self.targets)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published targets to topic: {self.targets}")

    def call_tsp_service_until_ready(self):
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for TSP service...")

        # Keep calling until we get a valid response
        while rclpy.ok():
            request = Trigger.Request()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response and response.success:
                try:
                    data = json.loads(response.message)
                    self.optimized_route = data['route']
                    self.get_logger().info(f"Optimized route received: {self.optimized_route}")
                    self.start_mission()
                    break
                except json.JSONDecodeError:
                    self.get_logger().warn("Invalid JSON received from TSP service. Retrying...")
            else:
                self.get_logger().warn(f"TSP service not ready: {response.message if response else 'No response'}")
                time.sleep(1)

    def go_to_house(self, start, target):
        self.get_logger().info(f"Navigating from {start} → {target}")
        subprocess.run([
            'ros2', 'run', 'my_robot_bringup', 'camera_follower',
            '--ros-args',
            '-p', f'target_house:={target}',
            '-p', f'start:={start}'
        ])

    def start_mission(self):
        current = 'PO'
        for house in self.optimized_route:
            self.go_to_house(current, house)
            current = house
            time.sleep(2)  # optional wait
        self.go_to_house(current, 'PO')
        self.get_logger().info("Mission complete!")

def main(args=None):
    rclpy.init(args=args)
    node = DeliverobotTSPClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
