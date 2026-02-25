#!/usr/bin/env python3
"""
Client for the named_goal_server service.
Usage: ros2 run smrr_navigation named_goal_client <location_name>
"""

import rclpy
from rclpy.node import Node
from smrr_interfaces.srv import GoToNamedPose
import sys


class NamedGoalClient(Node):
    def __init__(self):
        super().__init__('named_goal_client')
        self.client = self.create_client(GoToNamedPose, '/go_to_pose')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def send_request(self, location_name):
        request = GoToNamedPose.Request()
        request.name = location_name
        
        self.get_logger().info(f'Sending request to navigate to: {location_name}')
        future = self.client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            response = future.result()
            self.get_logger().info(f'Response - Accepted: {response.accepted}')
            self.get_logger().info(f'Message: {response.message}')
            return response
        else:
            self.get_logger().error('Service call failed')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print('Usage: ros2 run smrr_navigation named_goal_client <location_name>')
        print('Example locations: dock, ward_a, nurse_station, elevator')
        return
    
    location_name = sys.argv[1]
    
    client = NamedGoalClient()
    response = client.send_request(location_name)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
