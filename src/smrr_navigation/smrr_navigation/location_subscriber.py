#!/usr/bin/env python3
"""
Location Subscriber Bridge Node

This node subscribes to the 'location' topic (std_msgs/String) and the 'start_move'
topic (std_msgs/Bool). It stores the location and waits for a 'true' signal on
start_move before calling the named_goal_server's /go_to_pose service.

Usage:
    ros2 run smrr_navigation location_subscriber

Topic subscriptions:
    /location (std_msgs/String) - Receives and stores the target location name
    /start_move (std_msgs/Bool) - Triggers navigation when true is received

Service client: /go_to_pose (smrr_interfaces/GoToNamedPose)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from smrr_interfaces.srv import GoToNamedPose


class LocationSubscriber(Node):
    """
    Bridge node that connects a high-level location topic to the named_goal_server.
    Waits for start_move signal before initiating navigation.
    """
    
    def __init__(self):
        super().__init__('location_subscriber')
        
        # Declare parameters
        self.declare_parameter('location_topic', 'location')
        self.declare_parameter('start_move_topic', 'start_move')
        self.declare_parameter('service_name', '/go_to_pose')
        self.declare_parameter('service_timeout', 5.0)
        
        # Get parameters
        location_topic = self.get_parameter('location_topic').value
        start_move_topic = self.get_parameter('start_move_topic').value
        service_name = self.get_parameter('service_name').value
        self.service_timeout = self.get_parameter('service_timeout').value
        
        # Initialize state variables
        self.pending_location = None  # Stores location until start_move is true
        
        # Create service client
        self.client = self.create_client(GoToNamedPose, service_name)
        
        # Create location subscriber
        self.location_subscription = self.create_subscription(
            String,
            location_topic,
            self.location_callback,
            10
        )
        
        # Create start_move subscriber
        self.start_move_subscription = self.create_subscription(
            Bool,
            start_move_topic,
            self.start_move_callback,
            10
        )
        
        self.get_logger().info(f'Location Subscriber started')
        self.get_logger().info(f'  Subscribing to location: {location_topic}')
        self.get_logger().info(f'  Subscribing to start_move: {start_move_topic}')
        self.get_logger().info(f'  Service client: {service_name}')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for /go_to_pose service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                '/go_to_pose service not available after 10 seconds. '
                'Will retry when location messages arrive.'
            )
        else:
            self.get_logger().info('/go_to_pose service connected')
    
    def location_callback(self, msg):
        """
        Callback when a location name is published on the topic.
        Stores the location and waits for start_move signal.
        """
        location_name = msg.data.strip()
        
        if not location_name:
            self.get_logger().warn('Received empty location name, ignoring')
            return
        
        # Store the location
        self.pending_location = location_name
        self.get_logger().info(f'Received location: "{location_name}" - Waiting for start_move signal...')
    
    def start_move_callback(self, msg):
        """
        Callback when start_move signal is received.
        Initiates navigation if signal is true and location is set.
        """
        if not msg.data:
            # Ignore false signals
            return
        
        if self.pending_location is None:
            self.get_logger().warn('Received start_move=true but no location set. Publish to /location first.')
            return
        
        location_name = self.pending_location
        self.get_logger().info(f'Start_move=true received! Initiating navigation to "{location_name}"')
        
        # Check if service is available
        if not self.client.service_is_ready():
            self.get_logger().warn(
                f'/go_to_pose service not available, cannot navigate to {location_name}'
            )
            # Try to reconnect
            self.get_logger().info('Attempting to reconnect to service...')
            if not self.client.wait_for_service(timeout_sec=self.service_timeout):
                self.get_logger().error('Service still not available, keeping location for next attempt')
                return
        
        # Create service request
        request = GoToNamedPose.Request()
        request.name = location_name
        
        # Clear pending location after sending request
        self.pending_location = None
        
        # Call service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(lambda f: self.service_response_callback(f, location_name))
    
    def service_response_callback(self, future, location_name):
        """
        Callback when service call completes.
        """
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info(
                    f'✓ Navigation to "{location_name}" accepted: {response.message}'
                )
            else:
                self.get_logger().warn(
                    f'✗ Navigation to "{location_name}" rejected: {response.message}'
                )
        except Exception as e:
            self.get_logger().error(
                f'Service call to navigate to "{location_name}" failed: {str(e)}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = LocationSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception in location_subscriber: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
