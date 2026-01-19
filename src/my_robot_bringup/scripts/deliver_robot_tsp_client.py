#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
import json
import time

class DeliverobotTSPClient(Node):
    def __init__(self):
        super().__init__('deliverobot_tsp_client')
        self.client = self.create_client(Trigger, 'calculate_tsp_route')

    def call_response(self):
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for TSP service...')

        # Keep calling until a valid response is returned
        while rclpy.ok():
            request = Trigger.Request()
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response.success:
                data = json.loads(response.message)
                self.get_logger().info(f"Full Path (including intermediate nodes) optimised: {data['route']}")
                break
            else:
                self.get_logger().warn(f"{response.message}")
                self.get_logger().info("Retrying in 1 second...")
                time.sleep(1)

    def callback_response(self, future):
        response = future.result()
        while not response.success:
            self.get_logger().warn(f"TSP service not ready: {response.message}")
        
        data = json.loads(response.message)
        # self.get_logger().info(f"TSP {data['targets_order']}")
        self.get_logger().info(f"Full Path (including intermediate nodes) optimised: {data['route']}")
        # self.get_logger().info(f"Total Distance: {data['total_distance']}")

def main(args=None):
    rclpy.init(args=args)
    node = DeliverobotTSPClient()
    node.call_response()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
