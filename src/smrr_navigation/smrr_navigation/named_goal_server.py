#!/usr/bin/env python3
"""
Named Goal Server Node

This node provides a service to navigate to named locations stored in a YAML file.
It acts as a resolver and dispatcher, forwarding navigation requests to the multi-floor executor.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from smrr_interfaces.srv import GoToNamedPose, StartMission
from smrr_interfaces.action import NavigateToNamedLocation
from std_msgs.msg import String, Bool
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math
import uuid


class NamedGoalServer(Node):
    """
    A service server that resolves named locations and dispatches navigation
    to the multi-floor executor.
    """
    
    def __init__(self):
        super().__init__('named_goal_server')
        
        # Declare parameters
        self.declare_parameter('locations_file', 'locations.yaml')
        self.declare_parameter('multifloor_action_name', '/navigate_to_named_location')
        self.declare_parameter('multifloor_action_timeout', 10.0)
        self.declare_parameter('use_bt_mission_executor', True)
        self.declare_parameter('initial_floor_id', 'floor0')
        self.declare_parameter('start_mission_service_name', '/start_mission')
        self.declare_parameter('start_mission_timeout', 5.0)
        
        # Get parameters
        locations_file = self.get_parameter('locations_file').value
        self.multifloor_action_name = self.get_parameter('multifloor_action_name').value
        self.multifloor_action_timeout = self.get_parameter('multifloor_action_timeout').value
        self.use_bt_mission_executor = self.get_parameter('use_bt_mission_executor').value
        self.initial_floor_id = self.get_parameter('initial_floor_id').value
        self.start_mission_service_name = self.get_parameter('start_mission_service_name').value
        self.start_mission_timeout = self.get_parameter('start_mission_timeout').value
        
        # Load locations from YAML (floor-aware structure)
        self.floors, self.location_index = self.load_locations(locations_file)
        
        if not self.floors:
            self.get_logger().error('No locations loaded! Check your YAML file.')
        else:
            total_locations = sum(len(floor_data['locations']) for floor_data in self.floors.values())
            self.get_logger().info(f'Loaded {total_locations} named locations across {len(self.floors)} floor(s):')
            for floor_id, floor_data in self.floors.items():
                self.get_logger().info(f'  Floor: {floor_id}')
                for name, loc in floor_data['locations'].items():
                    self.get_logger().info(f'    - {name}: ({loc["x"]:.2f}, {loc["y"]:.2f}, {math.degrees(loc["yaw"]):.1f}°)')
        
        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Create service
        self.srv = self.create_service(
            GoToNamedPose,
            '/go_to_pose',
            self.handle_go_to_pose,
            callback_group=self.callback_group
        )
        
        # Create action client for multifloor executor (legacy mode)
        self.multifloor_client = ActionClient(
            self,
            NavigateToNamedLocation,
            self.multifloor_action_name,
            callback_group=self.callback_group
        )
        
        # Create service client for BT mission executor (new mode)
        self.start_mission_client = self.create_client(
            StartMission,
            self.start_mission_service_name,
            callback_group=self.callback_group
        )
        
        if self.use_bt_mission_executor:
            self.get_logger().info(f'BT Mission Executor mode enabled. Service: {self.start_mission_service_name}')
            self.get_logger().info(f'Initial floor: {self.initial_floor_id}')
        else:
            self.get_logger().info(f'Legacy action mode enabled. Action: {self.multifloor_action_name}')

        # Publish initial_floor_id on startup with transient-local QoS so any
        # late-joining subscriber (e.g. after restart) receives the current value.
        floor_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._floor_pub = self.create_publisher(String, '/current_floor_id', floor_qos)
        self._floor_sub = self.create_subscription(
            String,
            '/current_floor_id',
            self._current_floor_cb,
            floor_qos,
            callback_group=self.callback_group,
        )
        # Publish once at startup so stale data is cleared immediately
        self._publish_floor(self.initial_floor_id)

        # Publisher for /arrived — signals successful arrival at destination
        # Default QoS: RELIABLE, VOLATILE, KEEP_LAST depth=10
        self._arrived_pub = self.create_publisher(Bool, '/arrived', 10)
        
        self.get_logger().info('Named Goal Server ready. Service: /go_to_pose')

    def _publish_floor(self, floor_id: str):
        msg = String()
        msg.data = floor_id
        self._floor_pub.publish(msg)

    def _current_floor_cb(self, msg: String):
        """Update tracked floor whenever the BT publishes a new current floor."""
        new_floor = msg.data.strip()
        if new_floor and new_floor != self.initial_floor_id:
            self.get_logger().info(
                f'[current_floor_id] Floor updated: {self.initial_floor_id} -> {new_floor}')
            self.initial_floor_id = new_floor
    
    def load_locations(self, filename):
        """
        Load named locations from YAML file with floor-aware structure.
        Returns:
            floors: {floor_id: {"locations": {name: {x, y, yaw}}}}
            location_index: {name: [list of floor_ids that contain this name]}
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
            
            # Validate top-level structure
            if 'floors' not in data:
                self.get_logger().error('YAML file does not contain "floors" key')
                return {}, {}
            
            if not isinstance(data['floors'], dict):
                self.get_logger().error('"floors" must be a dictionary')
                return {}, {}
            
            floors = {}
            location_index = {}  # {location_name: [floor_id1, floor_id2, ...]}
            
            # Parse each floor
            for floor_id, floor_data in data['floors'].items():
                if not isinstance(floor_data, dict):
                    self.get_logger().warn(f'Floor "{floor_id}" is not a dictionary. Skipping.')
                    continue
                
                if 'locations' not in floor_data:
                    self.get_logger().warn(f'Floor "{floor_id}" does not contain "locations" key. Skipping.')
                    continue
                
                if not isinstance(floor_data['locations'], dict):
                    self.get_logger().warn(f'Floor "{floor_id}" locations is not a dictionary. Skipping.')
                    continue
                
                # Parse locations for this floor
                floor_locations = {}
                for name, pose_data in floor_data['locations'].items():
                    if not isinstance(pose_data, dict):
                        self.get_logger().warn(f'Location "{name}" on floor "{floor_id}" is not a dictionary. Skipping.')
                        continue
                    
                    if 'x' in pose_data and 'y' in pose_data and 'yaw' in pose_data:
                        try:
                            floor_locations[name] = {
                                'x': float(pose_data['x']),
                                'y': float(pose_data['y']),
                                'yaw': float(pose_data['yaw'])
                            }
                            
                            # Build reverse index
                            if name not in location_index:
                                location_index[name] = []
                            location_index[name].append(floor_id)
                            
                        except (ValueError, TypeError) as e:
                            self.get_logger().warn(f'Invalid numeric values for "{name}" on floor "{floor_id}": {e}')
                    else:
                        self.get_logger().warn(f'Location "{name}" on floor "{floor_id}" missing x, y, or yaw. Skipping.')
                
                if floor_locations:
                    floors[floor_id] = {'locations': floor_locations}
            
            return floors, location_index
            
        except FileNotFoundError:
            self.get_logger().error(f'Locations file not found: {filename}')
            return {}, {}
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')
            return {}, {}
        except Exception as e:
            self.get_logger().error(f'Unexpected error loading locations: {e}')
            return {}, {}
    
    def resolve_location(self, name):
        """
        Resolve a location name to (floor_id, pose).
        
        Args:
            name: Location name (string)
        
        Returns:
            tuple: (floor_id, pose_dict) where pose_dict contains {x, y, yaw}
            tuple: (None, error_message) if resolution fails
        """
        # Strip whitespace
        name = name.strip()
        
        # Reject empty string
        if not name:
            return None, 'Location name cannot be empty'
        
        # Check if location exists
        if name not in self.location_index:
            # Build list of all available locations
            all_locations = list(self.location_index.keys())
            return None, f'Unknown location: "{name}". Available locations: {all_locations}'
        
        # Get floors that contain this location
        matching_floors = self.location_index[name]
        
        # Check for ambiguity
        if len(matching_floors) > 1:
            return None, f'Ambiguous location: "{name}" exists on multiple floors: {matching_floors}. Please specify which floor.'
        
        # Exactly one match - resolve it
        floor_id = matching_floors[0]
        pose = self.floors[floor_id]['locations'][name]
        
        return floor_id, pose
    
    def handle_go_to_pose(self, request, response):
        """
        Service callback: resolve named location and dispatch.
        
        Behavior depends on use_bt_mission_executor parameter:
        - If True: Calls /start_mission service (BT-based execution)
        - If False: Uses legacy action client (action-based execution)
        
        For BT mode:
        - response.accepted reflects whether service accepted the request
        - response.message includes mission_id and BT result
        """
        location_name = request.name
        
        self.get_logger().info(f'Received request to navigate to: "{location_name}"')
        
        # Resolve location to (floor_id, pose)
        floor_id, result = self.resolve_location(location_name)
        
        if floor_id is None:
            # Resolution failed - result contains error message
            response.accepted = False
            response.message = result
            self.get_logger().warn(response.message)
            return response
        
        # Resolution succeeded - result contains pose
        loc = result
        
        # Log resolved floor and pose
        self.get_logger().info(f'Resolved location "{location_name}" -> floor: "{floor_id}", '
                              f'pose: ({loc["x"]:.2f}, {loc["y"]:.2f}, yaw: {loc["yaw"]:.2f} rad / {math.degrees(loc["yaw"]):.1f}°)')
        
        # Route to appropriate executor
        if self.use_bt_mission_executor:
            return self.handle_bt_mission_executor(request, response, location_name, floor_id, loc)
        else:
            return self.handle_legacy_action_executor(request, response, location_name, floor_id, loc)
    
    def handle_bt_mission_executor(self, request, response, location_name, floor_id, loc):
        """
        Call /start_mission service with resolved location data.
        
        Response mapping:
        - accepted=True if service call succeeded (regardless of BT result)
        - message includes mission_id and indicates BT success/failure
        """
        # Check if service is available
        if not self.start_mission_client.wait_for_service(timeout_sec=self.start_mission_timeout):
            response.accepted = False
            response.message = f'StartMission service {self.start_mission_service_name} not available'
            self.get_logger().error(response.message)
            return response
        
        # Generate mission ID
        mission_id = str(uuid.uuid4())
        
        # Build service request
        mission_request = StartMission.Request()
        mission_request.mission_id = mission_id
        mission_request.current_floor_id = self.initial_floor_id
        mission_request.target_floor_id = floor_id
        mission_request.target_location_name = location_name
        mission_request.x = float(loc['x'])
        mission_request.y = float(loc['y'])
        mission_request.yaw = float(loc['yaw'])
        
        self.get_logger().info(
            f'Calling StartMission service: mission_id={mission_id}, '
            f'current_floor={self.initial_floor_id}, target_floor={floor_id}'
        )
        
        try:
            # Call service synchronously with timeout
            future = self.start_mission_client.call_async(mission_request)
            
            # Wait for response with timeout
            import time
            start_time = time.time()
            timeout = 1500.0  # Match BT timeout + overhead
            
            while not future.done():
                if time.time() - start_time > timeout:
                    response.accepted = False
                    response.message = f'StartMission service call timeout for mission {mission_id}'
                    self.get_logger().error(response.message)
                    return response
                time.sleep(0.1)
            
            mission_response = future.result()
            
            # Map service response to our response
            # Note: We set accepted=True if the service accepted the request,
            # even if BT failed. The message will indicate BT result.
            if mission_response.accepted:
                response.accepted = True
                if mission_response.success:
                    # Update tracked floor so the next mission uses the correct current_floor_id
                    if self.initial_floor_id != floor_id:
                        self.get_logger().info(
                            f'Floor updated: {self.initial_floor_id} -> {floor_id}'
                        )
                    self.initial_floor_id = floor_id
                    response.message = f'Navigation to {location_name} completed: {mission_response.message}'
                    self.get_logger().info(response.message)
                    arrived_msg = Bool()
                    arrived_msg.data = True
                    self._arrived_pub.publish(arrived_msg)
                    self.get_logger().info(f'Published /arrived = True (reached: {location_name})')
                else:
                    # BT ran but failed (e.g., cross-floor not implemented, navigation failed)
                    response.message = f'Navigation to {location_name} failed: {mission_response.message}'
                    self.get_logger().warn(response.message)
            else:
                # Service rejected the request (invalid data)
                response.accepted = False
                response.message = f'StartMission rejected: {mission_response.message}'
                self.get_logger().error(response.message)
            
            return response
            
        except Exception as e:
            response.accepted = False
            response.message = f'Error calling StartMission service: {str(e)}'
            self.get_logger().error(response.message)
            return response
    
    def handle_legacy_action_executor(self, request, response, location_name, floor_id, loc):
        """
        Legacy action-based executor (original implementation).
        Dispatches to multifloor action server and returns immediately.
        """
        # Check if multifloor action server is available
        if not self.multifloor_client.wait_for_server(timeout_sec=self.multifloor_action_timeout):
            response.accepted = False
            response.message = f'Multifloor action server {self.multifloor_action_name} not available'
            self.get_logger().error(response.message)
            return response
        
        # Create multifloor navigation goal
        goal_msg = NavigateToNamedLocation.Goal()
        goal_msg.location_name = location_name
        goal_msg.target_floor_id = floor_id
        goal_msg.x = float(loc['x'])
        goal_msg.y = float(loc['y'])
        goal_msg.yaw = float(loc['yaw'])
        
        self.get_logger().info(
            f'Dispatching to multifloor executor: '
            f'action="{self.multifloor_action_name}", '
            f'location="{location_name}", floor="{floor_id}", '
            f'pose=({loc["x"]:.2f}, {loc["y"]:.2f}, {math.degrees(loc["yaw"]):.1f}°)'
        )
        
        try:
            # Send goal without blocking
            send_goal_future = self.multifloor_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(
                lambda future: self.multifloor_goal_response_callback(future, location_name)
            )
            
            # Return immediately - don't wait for navigation to complete
            response.accepted = True
            response.message = f'Dispatched navigation to multifloor executor for {location_name}'
            self.get_logger().info(response.message)
            return response
            
        except Exception as e:
            response.accepted = False
            response.message = f'Error dispatching to multifloor executor: {str(e)}'
            self.get_logger().error(response.message)
            return response
    
    def multifloor_goal_response_callback(self, future, location_name):
        """Callback when multifloor executor accepts/rejects goal"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f'Multifloor executor rejected goal to {location_name}')
                return
            
            self.get_logger().info(f'Multifloor executor accepted goal to {location_name}')
            
            # Register callback for when navigation completes
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda future: self.multifloor_result_callback(future, location_name)
            )
        except Exception as e:
            self.get_logger().error(f'Error in multifloor goal response for {location_name}: {e}')
    
    def multifloor_result_callback(self, future, location_name):
        """Callback when multifloor navigation completes"""
        try:
            result = future.result()
            action_result = result.result
            
            if action_result.success:
                self.get_logger().info(
                    f'Multifloor navigation to {location_name} succeeded: {action_result.message}'
                )
            else:
                self.get_logger().warn(
                    f'Multifloor navigation to {location_name} failed (status={action_result.nav_status}): '
                    f'{action_result.message}'
                )
        except Exception as e:
            self.get_logger().error(f'Error in multifloor result callback for {location_name}: {e}')


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
