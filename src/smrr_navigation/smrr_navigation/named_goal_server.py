#!/usr/bin/env python3
"""
Named Goal Server Node

This node provides a service to navigate to named locations stored in a YAML file.
It acts as a bridge between simple string-based location names and Nav2's NavigateToPose action.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from smrr_interfaces.srv import GoToNamedPose
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math


class NamedGoalServer(Node):
    """
    A service server that navigates to named locations using Nav2.
    """
    
    def __init__(self):
        super().__init__('named_goal_server')
        
        # Declare parameters
        self.declare_parameter('locations_file', 'locations.yaml')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('action_timeout', 300.0)  # 5 minutes default
        
        # Get parameters
        locations_file = self.get_parameter('locations_file').value
        self.global_frame = self.get_parameter('global_frame').value
        self.action_timeout = self.get_parameter('action_timeout').value
        
        # Load locations from YAML
        self.locations = self.load_locations(locations_file)
        
        if not self.locations:
            self.get_logger().error('No locations loaded! Check your YAML file.')
        else:
            self.get_logger().info(f'Loaded {len(self.locations)} named locations:')
            for name in self.locations.keys():
                loc = self.locations[name]
                self.get_logger().info(f'  - {name}: ({loc["x"]:.2f}, {loc["y"]:.2f}, {math.degrees(loc["yaw"]):.1f}°)')
        
        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Create service
        self.srv = self.create_service(
            GoToNamedPose,
            '/go_to_pose',
            self.handle_go_to_pose,
            callback_group=self.callback_group
        )
        
        # Create action client for Nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Wait for action server
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('navigate_to_pose action server not available after 10 seconds')
        else:
            self.get_logger().info('navigate_to_pose action server connected')
        
        self.get_logger().info('Named Goal Server ready. Service: /go_to_pose')
    
    def load_locations(self, filename):
        """
        Load named locations from YAML file.
        Returns a dictionary: {name: {x, y, yaw}}
        """
        try:
            # Try to load from config directory in package share
            config_dir = os.path.join(
                get_package_share_directory('smrr_navigation'),
                'config'
            )
            filepath = os.path.join(config_dir, filename)
            
            if not os.path.exists(filepath):
                # Try relative path from workspace
                self.get_logger().warn(f'File not found at {filepath}, trying relative path')
                filepath = os.path.join(
                    os.path.dirname(__file__),
                    '..',
                    'config',
                    filename
                )
            
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'locations' not in data:
                self.get_logger().error('YAML file does not contain "locations" key')
                return {}
            
            locations = {}
            for name, pose_data in data['locations'].items():
                if 'x' in pose_data and 'y' in pose_data and 'yaw' in pose_data:
                    locations[name] = {
                        'x': float(pose_data['x']),
                        'y': float(pose_data['y']),
                        'yaw': float(pose_data['yaw'])
                    }
                else:
                    self.get_logger().warn(f'Location "{name}" missing x, y, or yaw. Skipping.')
            
            return locations
            
        except FileNotFoundError:
            self.get_logger().error(f'Locations file not found: {filename}')
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')
            return {}
        except Exception as e:
            self.get_logger().error(f'Unexpected error loading locations: {e}')
            return {}
    
    def create_pose_stamped(self, x, y, yaw):
        """
        Create a PoseStamped message from x, y, yaw coordinates.
        """
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def handle_go_to_pose(self, request, response):
        """
        Service callback: look up named location and send Nav2 action goal.
        Returns immediately after goal is accepted (non-blocking).
        """
        location_name = request.name
        
        self.get_logger().info(f'Received request to navigate to: {location_name}')
        
        # Check if location exists
        if location_name not in self.locations:
            response.accepted = False
            response.message = f'Unknown location: {location_name}. Available: {list(self.locations.keys())}'
            self.get_logger().warn(response.message)
            return response
        
        # Get location data
        loc = self.locations[location_name]
        
        # Check if action server is available
        if not self.nav_client.server_is_ready():
            response.accepted = False
            response.message = 'Navigation action server not available'
            self.get_logger().error(response.message)
            return response
        
        # Create Nav2 action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(loc['x'], loc['y'], loc['yaw'])
        
        # Send action goal asynchronously
        self.get_logger().info(f'Sending navigation goal to ({loc["x"]:.2f}, {loc["y"]:.2f}, {math.degrees(loc["yaw"]):.1f}°)')
        
        try:
            # Send goal without blocking
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )
            send_goal_future.add_done_callback(
                lambda future: self.goal_response_callback(future, location_name)
            )
            
            # Return immediately - don't wait for navigation to complete
            response.accepted = True
            response.message = f'Navigation goal to {location_name} sent successfully'
            self.get_logger().info(response.message)
            return response
            
        except Exception as e:
            response.accepted = False
            response.message = f'Error sending navigation goal: {str(e)}'
            self.get_logger().error(response.message)
            return response
    
    def navigation_feedback_callback(self, feedback_msg):
        """Callback for navigation feedback (optional logging)"""
        feedback = feedback_msg.feedback
        # Optionally log distance remaining, time elapsed, etc.
        # self.get_logger().info(f'Navigation feedback: {feedback}', throttle_duration_sec=5.0)
    
    def goal_response_callback(self, future, location_name):
        """Callback when goal is accepted/rejected by action server"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal to {location_name} was rejected by action server')
            return
        
        self.get_logger().info(f'Goal to {location_name} accepted by action server')
        
        # Register callback for when navigation completes
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.navigation_result_callback(future, location_name)
        )
    
    def navigation_result_callback(self, future, location_name):
        """Callback when navigation completes"""
        result = future.result()
        status = result.status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Successfully navigated to {location_name}')
        elif status == 5:  # CANCELED
            self.get_logger().warn(f'Navigation to {location_name} was canceled')
        elif status == 6:  # ABORTED
            self.get_logger().error(f'Navigation to {location_name} was aborted')
        else:
            self.get_logger().warn(f'Navigation to {location_name} completed with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    
    node = NamedGoalServer()
    
    # Use MultiThreadedExecutor for concurrent service and action handling
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception in named_goal_server: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
