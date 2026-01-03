#!/usr/bin/env python3
"""
SMRR Multi-Floor BT Navigator Node

This node provides an action server for NavigateToNamedLocation that wraps Nav2's NavigateToPose.
For Step 4a, it only supports same-floor navigation by forwarding goals to Nav2.

Action server: /navigate_to_named_location (smrr_interfaces/action/NavigateToNamedLocation)
Action client: navigate_to_pose (nav2_msgs/action/NavigateToPose)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from smrr_interfaces.action import NavigateToNamedLocation
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math
import time
import asyncio
import subprocess


class SMRRMultiFloorBTNavigator(Node):
    """
    Action server that handles multi-floor navigation requests.
    Step 4a: Only same-floor navigation via Nav2 forwarding.
    """
    
    def __init__(self):
        super().__init__('smrr_multifloor_bt_navigator')
        
        # Declare parameters
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('nav2_action_name', 'navigate_to_pose')
        self.declare_parameter('nav2_wait_timeout', 10.0)
        self.declare_parameter('feedback_rate_hz', 2.0)
        self.declare_parameter('initial_floor_id', 'floor0')
        self.declare_parameter('locations_file', 'locations.yaml')
        
        # Gazebo elevator control parameters
        self.declare_parameter('gz_cli', 'gz-11.14.0')
        self.declare_parameter('gz_elevator_topic', '/gazebo/default/elevator')
        
        # Door-open detection parameters (LaserScan only)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('door_window_center_deg', 0.0)
        self.declare_parameter('door_window_width_deg', 30.0)
        self.declare_parameter('door_open_range_threshold', 2.0)
        self.declare_parameter('door_open_fraction_threshold', 0.6)
        self.declare_parameter('door_open_stable_time', 1.0)
        self.declare_parameter('door_open_timeout', 30.0)
        self.declare_parameter('door_poll_rate_hz', 10.0)
        
        # Guarded cmd_vel motion parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('enter_speed', 1.0)
        self.declare_parameter('enter_distance', 1.5)
        self.declare_parameter('min_front_clearance', 0.35)
        self.declare_parameter('front_check_window_deg', 20.0)
        self.declare_parameter('motion_control_rate', 20.0)
        self.declare_parameter('motion_timeout_padding', 3.0)
        
        # Map switching parameters
        self.declare_parameter('map_server_load_map_service', '/map_server/load_map')
        self.declare_parameter('map_switch_timeout', 10.0)
        
        # Costmap clearing parameters
        self.declare_parameter('clear_costmaps_after_map_switch', True)
        self.declare_parameter('global_clear_costmap_service', '/global_costmap/clear_entirely_global_costmap')
        self.declare_parameter('local_clear_costmap_service', '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('clear_costmap_timeout', 3.0)
        
        # AMCL settle check parameters
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('amcl_settle_min_msgs', 3)
        self.declare_parameter('amcl_settle_timeout', 5.0)
        self.declare_parameter('amcl_pose_stale_sec', 1.0)
        
        # Get parameters
        self.global_frame = self.get_parameter('global_frame').value
        self.nav2_action_name = self.get_parameter('nav2_action_name').value
        self.nav2_wait_timeout = self.get_parameter('nav2_wait_timeout').value
        self.feedback_rate_hz = self.get_parameter('feedback_rate_hz').value
        self.current_floor_id = self.get_parameter('initial_floor_id').value
        self.current_map_mode = 'closed'  # Track current map mode: 'open' or 'closed'
        locations_file = self.get_parameter('locations_file').value
        
        # Gazebo elevator control parameters
        self.gz_cli = self.get_parameter('gz_cli').value
        self.gz_elevator_topic = self.get_parameter('gz_elevator_topic').value
        
        # Door-open detection parameters
        self.scan_topic = self.get_parameter('scan_topic').value
        self.door_window_center_deg = self.get_parameter('door_window_center_deg').value
        self.door_window_width_deg = self.get_parameter('door_window_width_deg').value
        self.door_open_range_threshold = self.get_parameter('door_open_range_threshold').value
        self.door_open_fraction_threshold = self.get_parameter('door_open_fraction_threshold').value
        self.door_open_stable_time = self.get_parameter('door_open_stable_time').value
        self.door_open_timeout = self.get_parameter('door_open_timeout').value
        self.door_poll_rate_hz = self.get_parameter('door_poll_rate_hz').value
        
        # Guarded cmd_vel motion parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.enter_speed = self.get_parameter('enter_speed').value
        self.enter_distance = self.get_parameter('enter_distance').value
        self.min_front_clearance = self.get_parameter('min_front_clearance').value
        self.front_check_window_deg = self.get_parameter('front_check_window_deg').value
        self.motion_control_rate = self.get_parameter('motion_control_rate').value
        self.motion_timeout_padding = self.get_parameter('motion_timeout_padding').value
        
        # Map switching parameters
        self.map_server_load_map_service = self.get_parameter('map_server_load_map_service').value
        self.map_switch_timeout = self.get_parameter('map_switch_timeout').value
        
        # Costmap clearing parameters
        self.clear_costmaps_after_map_switch = self.get_parameter('clear_costmaps_after_map_switch').value
        self.global_clear_costmap_service = self.get_parameter('global_clear_costmap_service').value
        self.local_clear_costmap_service = self.get_parameter('local_clear_costmap_service').value
        self.clear_costmap_timeout = self.get_parameter('clear_costmap_timeout').value
        
        # AMCL settle check parameters
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.amcl_settle_min_msgs = self.get_parameter('amcl_settle_min_msgs').value
        self.amcl_settle_timeout = self.get_parameter('amcl_settle_timeout').value
        self.amcl_pose_stale_sec = self.get_parameter('amcl_pose_stale_sec').value
        
        # Load floor-aware locations configuration
        self.floors_config = self.load_floors_config(locations_file)
        
        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publisher for current floor ID
        self.floor_id_publisher = self.create_publisher(
            String,
            '/current_floor_id',
            10
        )
        
        # Create publisher for initial pose (AMCL)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Create cmd_vel publisher for guarded motion
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        
        # Create LaserScan subscriber for door detection
        self.latest_scan = None
        self.latest_scan_time = None
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        
        # Create Odometry subscriber for guarded motion
        self.latest_odom = None
        self.latest_odom_time = None
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        # Create AMCL pose subscriber for settle check
        self.latest_amcl_pose = None
        self.latest_amcl_time = None
        self.amcl_seq = 0
        self.amcl_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_callback,
            10
        )
        
        # Create Nav2 action client
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            self.nav2_action_name,
            callback_group=self.callback_group
        )
        
        # Create service clients for map switching
        self.load_map_client = self.create_client(
            LoadMap,
            self.map_server_load_map_service
        )
        
        self.global_clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.global_clear_costmap_service
        )
        
        self.local_clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.local_clear_costmap_service
        )
        
        # Create action server for multi-floor navigation
        self.action_server = ActionServer(
            self,
            NavigateToNamedLocation,
            '/navigate_to_named_location',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('SMRR Multi-Floor BT Navigator initialized')
        self.get_logger().info(f'  Action server: /navigate_to_named_location')
        self.get_logger().info(f'  Nav2 client: {self.nav2_action_name}')
        self.get_logger().info(f'  Global frame: {self.global_frame}')
        self.get_logger().info(f'  Current floor: {self.current_floor_id}')
        
        # Publish initial floor ID
        self.publish_current_floor()
        
        # Track active Nav2 goal for cancellation
        self.active_nav2_goal_handle = None
    
    def load_floors_config(self, filename):
        """
        Load floor-aware locations configuration from YAML file.
        Returns dictionary: {floor_id: {map_yaml, locations}}
        """
        try:
            # Try to load from config directory in package share
            config_dir = os.path.join(
                get_package_share_directory('smrr_navigation'),
                'config'
            )
            filepath = os.path.join(config_dir, filename)
            
            if not os.path.exists(filepath):
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
                return {}
            
            if not isinstance(data['floors'], dict):
                self.get_logger().error('"floors" must be a dictionary')
                return {}
            
            floors_config = {}
            
            # Parse and validate each floor
            for floor_id, floor_data in data['floors'].items():
                if not isinstance(floor_data, dict):
                    self.get_logger().warn(f'Floor "{floor_id}" is not a dictionary. Skipping.')
                    continue
                
                # Support both old (map_yaml) and new (maps.closed/open) formats
                maps_config = {}
                if 'maps' in floor_data and isinstance(floor_data['maps'], dict):
                    # New format with separate closed/open maps
                    if 'closed' in floor_data['maps']:
                        maps_config['closed'] = floor_data['maps']['closed']
                    if 'open' in floor_data['maps']:
                        maps_config['open'] = floor_data['maps']['open']
                    
                    if not maps_config:
                        self.get_logger().error(f'Floor "{floor_id}" has "maps" but no "closed" or "open" keys. Skipping.')
                        continue
                    
                    self.get_logger().info(f'Floor "{floor_id}" using new maps format (closed/open)')
                    
                elif 'map_yaml' in floor_data:
                    # Old format - use map_yaml for both closed and open (with warning)
                    maps_config['closed'] = floor_data['map_yaml']
                    maps_config['open'] = floor_data['map_yaml']
                    self.get_logger().warn(
                        f'Floor "{floor_id}" using old format (map_yaml). '
                        f'Open map missing - using same map for closed and open. '
                        f'Consider updating to maps.closed/maps.open format.'
                    )
                else:
                    self.get_logger().error(f'Floor "{floor_id}" missing both "maps" and "map_yaml". Skipping.')
                    continue
                
                if 'locations' not in floor_data or not isinstance(floor_data['locations'], dict):
                    self.get_logger().error(f'Floor "{floor_id}" missing or invalid "locations". Skipping.')
                    continue
                
                # Validate required locations
                if 'elevator_staging' not in floor_data['locations']:
                    self.get_logger().error(f'Floor "{floor_id}" missing required "elevator_staging" location. Skipping.')
                    continue
                
                if 'elevator_exit' not in floor_data['locations']:
                    self.get_logger().error(f'Floor "{floor_id}" missing required "elevator_exit" location. Skipping.')
                    continue
                
                # Store validated floor config
                floors_config[floor_id] = {
                    'maps': maps_config,
                    'locations': floor_data['locations']
                }
                
                # Store amcl_initial_pose fields if defined (support both old and new format)
                if 'amcl_initial_pose_closed' in floor_data:
                    floors_config[floor_id]['amcl_initial_pose_closed'] = floor_data['amcl_initial_pose_closed']
                if 'amcl_initial_pose_open' in floor_data:
                    floors_config[floor_id]['amcl_initial_pose_open'] = floor_data['amcl_initial_pose_open']
                # Legacy support for old 'amcl_initial_pose' field
                if 'amcl_initial_pose' in floor_data:
                    floors_config[floor_id]['amcl_initial_pose'] = floor_data['amcl_initial_pose']
                
                self.get_logger().info(f'Loaded floor "{floor_id}" with {len(floor_data["locations"])} locations')
            
            if not floors_config:
                self.get_logger().error('No valid floors loaded from YAML!')
            
            return floors_config
            
        except FileNotFoundError:
            self.get_logger().error(f'Locations file not found: {filename}')
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')
            return {}
        except Exception as e:
            self.get_logger().error(f'Unexpected error loading floors config: {e}')
            return {}
    
    def get_floor_map_yaml(self, floor_id, mode='closed'):
        """
        Get map YAML filename for a floor.
        Args:
            floor_id: Floor identifier (e.g., 'floor0')
            mode: 'closed' or 'open'
        Returns:
            Absolute path to map YAML file, or None if not found
        """
        if floor_id not in self.floors_config:
            self.get_logger().error(f'Floor "{floor_id}" not found in config')
            return None
        
        maps_config = self.floors_config[floor_id].get('maps', {})
        if mode not in maps_config:
            self.get_logger().error(f'Map mode "{mode}" not found for floor "{floor_id}"')
            return None
        
        map_filename = maps_config[mode]
        
        # If it's already an absolute path, use it
        if os.path.isabs(map_filename):
            return map_filename
        
        # Otherwise, resolve relative to package maps directory
        maps_dir = os.path.join(
            get_package_share_directory('smrr_navigation'),
            'maps'
        )
        map_path = os.path.join(maps_dir, map_filename)
        
        return map_path
    
    async def load_map(self, map_yaml_path, timeout_sec):
        """
        Load a map using the map_server LoadMap service.
        Args:
            map_yaml_path: Absolute path to map YAML file
            timeout_sec: Timeout for service call
        Returns:
            (success: bool, message: str)
        """
        # Wait for service availability
        self.get_logger().info(f'Waiting for LoadMap service: {self.map_server_load_map_service}')
        if not self.load_map_client.wait_for_service(timeout_sec=timeout_sec):
            msg = f'LoadMap service unavailable: {self.map_server_load_map_service}'
            self.get_logger().error(msg)
            return (False, msg)
        
        # Create request
        request = LoadMap.Request()
        request.map_url = map_yaml_path
        
        self.get_logger().info(f'Loading map: {map_yaml_path}')
        
        # Call service
        try:
            future = self.load_map_client.call_async(request)
            
            # Wait for response with timeout
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    msg = f'LoadMap service call timed out after {timeout_sec}s'
                    self.get_logger().error(msg)
                    return (False, msg)
                time.sleep(0.1)
            
            response = future.result()
            
            # Check response - LoadMap.Response has 'result' field (uint8)
            # Nav2's LoadMap returns SUCCESS=0 or INVALID_MAP_DATA=2 or other error codes
            # Treat result == 0 as success
            self.get_logger().info(f'LoadMap response.result = {response.result}')
            
            # Log additional fields if available
            if hasattr(response, 'map'):
                self.get_logger().info(f'LoadMap response has map field')
            if hasattr(response, 'error_string'):
                self.get_logger().info(f'LoadMap error_string: {response.error_string}')
            if hasattr(response, 'message'):
                self.get_logger().info(f'LoadMap message: {response.message}')
            
            if response.result == 0:
                self.get_logger().info(f'Map loaded successfully: {map_yaml_path}')
                return (True, 'Map loaded successfully')
            else:
                msg = f'LoadMap failed with result code: {response.result}'
                self.get_logger().error(msg)
                return (False, msg)
                
        except Exception as e:
            msg = f'Exception during LoadMap call: {str(e)}'
            self.get_logger().error(msg)
            return (False, msg)
    
    async def clear_costmaps(self, timeout_sec):
        """
        Clear global and local costmaps after map switch.
        Returns:
            (success: bool, message: str)
        """
        # Clear global costmap
        self.get_logger().info('Clearing global costmap...')
        if not self.global_clear_costmap_client.wait_for_service(timeout_sec=timeout_sec):
            msg = 'Global clear costmap service unavailable'
            self.get_logger().warn(msg)
            return (False, msg)
        
        try:
            request = ClearEntireCostmap.Request()
            future = self.global_clear_costmap_client.call_async(request)
            
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    self.get_logger().warn('Global costmap clear timed out')
                    break
                time.sleep(0.1)
            
            if future.done():
                self.get_logger().info('Global costmap cleared')
        except Exception as e:
            self.get_logger().warn(f'Exception clearing global costmap: {e}')
        
        # Clear local costmap
        self.get_logger().info('Clearing local costmap...')
        if not self.local_clear_costmap_client.wait_for_service(timeout_sec=timeout_sec):
            msg = 'Local clear costmap service unavailable'
            self.get_logger().warn(msg)
            return (False, msg)
        
        try:
            request = ClearEntireCostmap.Request()
            future = self.local_clear_costmap_client.call_async(request)
            
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout_sec:
                    self.get_logger().warn('Local costmap clear timed out')
                    break
                time.sleep(0.1)
            
            if future.done():
                self.get_logger().info('Local costmap cleared')
        except Exception as e:
            self.get_logger().warn(f'Exception clearing local costmap: {e}')
        
        return (True, 'Costmaps cleared')
    
    def get_named_pose(self, floor_id, location_key):
        """
        Get pose (x, y, yaw) for a named location on a specific floor.
        Returns tuple (x, y, yaw) or None if not found.
        """
        if floor_id not in self.floors_config:
            self.get_logger().error(f'Floor "{floor_id}" not found in config')
            return None
        
        locations = self.floors_config[floor_id].get('locations', {})
        if location_key not in locations:
            self.get_logger().error(f'Location "{location_key}" not found on floor "{floor_id}"')
            return None
        
        pose_data = locations[location_key]
        
        try:
            x = float(pose_data['x'])
            y = float(pose_data['y'])
            yaw = float(pose_data['yaw'])
            return (x, y, yaw)
        except (KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f'Invalid pose data for "{location_key}" on "{floor_id}": {e}')
            return None
    
    def get_amcl_initial_pose(self, floor_id, mode='closed'):
        """
        Get AMCL initial pose for a floor based on map mode.
        Args:
            floor_id: Floor identifier
            mode: 'open' or 'closed' map mode
        Returns:
            tuple (x, y, yaw) or None if not found
        """
        if floor_id not in self.floors_config:
            self.get_logger().error(f'Floor "{floor_id}" not found in config')
            return None
        
        # Try mode-specific pose first
        field_name = f'amcl_initial_pose_{mode}'
        amcl_pose_data = self.floors_config[floor_id].get(field_name, None)
        
        # Fallback to legacy field if mode-specific not found
        if amcl_pose_data is None:
            amcl_pose_data = self.floors_config[floor_id].get('amcl_initial_pose', None)
            if amcl_pose_data is not None:
                self.get_logger().warn(f'Using legacy amcl_initial_pose for floor "{floor_id}" (mode: {mode})')
        
        if amcl_pose_data is None:
            self.get_logger().warn(f'No amcl_initial_pose_{mode} defined for floor "{floor_id}"')
            return None
        
        try:
            x = float(amcl_pose_data['x'])
            y = float(amcl_pose_data['y'])
            yaw = float(amcl_pose_data['yaw'])
            return (x, y, yaw)
        except (KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f'Invalid amcl_initial_pose_{mode} data for floor "{floor_id}": {e}')
            return None
    
    def create_pose_stamped(self, x, y, yaw, frame_id="map"):
        """Create a PoseStamped message from x, y, yaw."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        quat = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose
    
    def publish_current_floor(self):
        """Publish current floor ID to /current_floor_id topic."""
        msg = String()
        msg.data = self.current_floor_id
        self.floor_id_publisher.publish(msg)
    
    def publish_initial_pose(self, x, y, yaw, frame_id="map"):
        """Publish initial pose for AMCL localization."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        quat = self.yaw_to_quaternion(yaw)
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]
        
        # Set covariance - small values indicate high confidence
        # Diagonal: [x, y, z, roll, pitch, yaw]
        pose_msg.pose.covariance[0] = 0.25  # x variance
        pose_msg.pose.covariance[7] = 0.25  # y variance
        pose_msg.pose.covariance[35] = 0.06853  # yaw variance (~15 degrees)
        
        self.initial_pose_publisher.publish(pose_msg)
        self.get_logger().info(f'Published initial pose: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')
    
    def goal_callback(self, goal_request):
        """Accept or reject incoming navigation goals."""
        # Validate goal parameters
        if not math.isfinite(goal_request.x) or not math.isfinite(goal_request.y) or not math.isfinite(goal_request.yaw):
            self.get_logger().warn(
                f'Rejecting goal with invalid coordinates: x={goal_request.x}, y={goal_request.y}, yaw={goal_request.yaw}'
            )
            return GoalResponse.REJECT
        
        if not goal_request.target_floor_id or goal_request.target_floor_id.strip() == '':
            self.get_logger().warn('Rejecting goal with empty target_floor_id')
            return GoalResponse.REJECT
        
        self.get_logger().info(
            f'Accepting navigation goal: location="{goal_request.location_name}", '
            f'floor="{goal_request.target_floor_id}", '
            f'pose=({goal_request.x:.2f}, {goal_request.y:.2f}, {goal_request.yaw:.2f})'
        )
        
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info(f'Received cancellation request for goal: {goal_handle.goal_id}')
        return CancelResponse.ACCEPT
    
    def yaw_to_quaternion(self, yaw):
        """
        Convert yaw angle to quaternion (2D rotation).
        Returns (x, y, z, w) tuple.
        """
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    
    def wait_for_nav2_server(self):
        """
        Wait for Nav2 action server to become available.
        Returns True if available, False if timeout.
        """
        self.get_logger().info(f'Waiting for Nav2 action server: {self.nav2_action_name}...')
        
        if self.nav2_client.wait_for_server(timeout_sec=self.nav2_wait_timeout):
            self.get_logger().info('Nav2 action server is available')
            return True
        else:
            self.get_logger().error(
                f'Nav2 action server not available after {self.nav2_wait_timeout} seconds'
            )
            return False
    
    async def execute_callback(self, goal_handle):
        """
        Execute the navigation goal with floor awareness.
        - Same floor: navigate directly to target
        - Different floor: navigate to elevator staging and stop
        """
        request = goal_handle.request
        
        self.get_logger().info(
            f'Executing navigation: location="{request.location_name}", '
            f'current_floor="{self.current_floor_id}", target_floor="{request.target_floor_id}", '
            f'pose=({request.x:.2f}, {request.y:.2f}, {request.yaw:.2f})'
        )
        
        # Prepare result
        result = NavigateToNamedLocation.Result()
        
        # Send initial feedback
        feedback = NavigateToNamedLocation.Feedback()
        feedback.state = 'ACCEPTED'
        feedback.active_floor_id = self.current_floor_id
        feedback.active_step = 'goal_accepted'
        goal_handle.publish_feedback(feedback)
        
        # Check if floors configuration is valid
        if not self.floors_config:
            result.success = False
            result.nav_status = 0
            result.message = 'Floors configuration not loaded'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result
        
        # Check if target floor exists
        if request.target_floor_id not in self.floors_config:
            result.success = False
            result.nav_status = 0
            result.message = f'Target floor "{request.target_floor_id}" not found in configuration'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result
        
        # CASE A: Same floor navigation
        if request.target_floor_id == self.current_floor_id:
            return await self._execute_same_floor_navigation(
                goal_handle, request, feedback, result
            )
        
        # CASE B: Different floor - navigate to elevator staging
        else:
            return await self._execute_cross_floor_staging(
                goal_handle, request, feedback, result
            )
    
    async def _execute_same_floor_navigation(self, goal_handle, request, feedback, result):
        """Execute navigation on the same floor."""
        self.get_logger().info(f'Same-floor navigation to ({request.x:.2f}, {request.y:.2f})')
        
        # Build PoseStamped from goal
        target_pose = self.create_pose_stamped(request.x, request.y, request.yaw)
        
        # Update feedback
        feedback.state = 'SAME_FLOOR_NAVIGATING'
        feedback.active_floor_id = self.current_floor_id
        feedback.active_step = 'NAVIGATE_TO_TARGET'
        goal_handle.publish_feedback(feedback)
        
        # Navigate to target pose
        nav_result = await self._navigate_to_pose(goal_handle, target_pose, feedback, request.location_name)
        
        if nav_result[0]:  # Success
            result.success = True
            result.nav_status = nav_result[1]
            result.message = nav_result[2]
            
            # Send final feedback
            feedback.state = 'DONE'
            feedback.active_step = 'completed'
            goal_handle.publish_feedback(feedback)
            
            goal_handle.succeed()
        else:  # Failed
            result.success = False
            result.nav_status = nav_result[1]
            result.message = nav_result[2]
            
            if nav_result[1] == 5:  # CANCELED
                goal_handle.canceled()
            else:
                goal_handle.abort()
        
        return result
    
    async def _execute_cross_floor_staging(self, goal_handle, request, feedback, result):
        """Navigate to elevator staging and execute full elevator entry sequence."""
        self.get_logger().info(
            f'Cross-floor navigation: current={self.current_floor_id}, target={request.target_floor_id}'
        )
        
        # Step 1: Get elevator staging pose for current floor
        staging_pose = self.get_named_pose(self.current_floor_id, 'elevator_staging')
        if staging_pose is None:
            result.success = False
            result.nav_status = 0
            result.message = f'elevator_staging not found on current floor "{self.current_floor_id}"'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result
        
        sx, sy, syaw = staging_pose
        self.get_logger().info(
            f'Navigating to elevator_staging on {self.current_floor_id}: ({sx:.2f}, {sy:.2f}, {math.degrees(syaw):.1f}°)'
        )
        
        # Build PoseStamped for staging
        staging_pose_stamped = self.create_pose_stamped(sx, sy, syaw)
        
        # Update feedback
        feedback.state = 'NAV_TO_ELEVATOR_STAGING'
        feedback.active_floor_id = self.current_floor_id
        feedback.active_step = 'NAVIGATE_TO_STAGING'
        goal_handle.publish_feedback(feedback)
        
        # Navigate to staging pose
        nav_result = await self._navigate_to_pose(goal_handle, staging_pose_stamped, feedback, 'elevator_staging')
        
        if not nav_result[0]:
            # Failed to reach staging
            result.success = False
            result.nav_status = nav_result[1]
            result.message = f'Failed to reach elevator staging: {nav_result[2]}'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result
        
        # Step 2: Switch to open map with auto initial pose
        self.get_logger().info('Reached elevator staging, switching to open map...')
        
        # _switch_map will auto-retrieve amcl_initial_pose_open from config
        switch_ok, switch_msg = await self._switch_map(
            self.current_floor_id, 'open', feedback, goal_handle
        )
        if not switch_ok:
            result.success = False
            result.nav_status = 0
            result.message = switch_msg
            self.get_logger().error(result.message)
            self.stop_robot()
            goal_handle.abort()
            return result
        
        # Step 3: Execute elevator entry sequence
        self.get_logger().info('Map switched to open, starting elevator entry sequence')
        ok, message = await self._do_elevator_entry_sequence(goal_handle, request, feedback)
        
        if not ok:
            result.success = False
            result.nav_status = 6  # ABORTED
            result.message = message
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result
        
        # Floor transition completed successfully
        result.success = True
        result.nav_status = 4  # SUCCEEDED
        result.message = message
        self.get_logger().info(result.message)
        
        # Send final feedback
        feedback.state = 'DONE'
        feedback.active_step = 'completed'
        feedback.active_floor_id = self.current_floor_id
        goal_handle.publish_feedback(feedback)
        
        goal_handle.succeed()
        
        return result
    
    async def wait_for_amcl_settle(self, timeout_sec=None):
        """
        Wait for AMCL to settle after map switch.
        Returns: (success: bool, message: str)
        """
        if timeout_sec is None:
            timeout_sec = self.amcl_settle_timeout
        
        self.get_logger().info(f'Waiting for AMCL to settle (min {self.amcl_settle_min_msgs} msgs, timeout {timeout_sec}s)...')
        
        start_seq = self.amcl_seq
        start_time = self.get_clock().now()
        last_log_time = start_time
        
        while True:
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            # Log progress every second
            if (self.get_clock().now() - last_log_time).nanoseconds / 1e9 >= 1.0:
                msgs_received = self.amcl_seq - start_seq
                if self.latest_amcl_time is not None:
                    amcl_age = (self.get_clock().now() - self.latest_amcl_time).nanoseconds / 1e9
                    self.get_logger().info(f'AMCL settle progress: {msgs_received} msgs, last age {amcl_age:.2f}s')
                else:
                    self.get_logger().warn(f'AMCL settle progress: {msgs_received} msgs, no AMCL messages yet')
                last_log_time = self.get_clock().now()
            
            if elapsed > timeout_sec:
                msgs_received = self.amcl_seq - start_seq
                
                # If we received at least 1 message and it's reasonably fresh, accept it
                if msgs_received >= 1 and self.latest_amcl_time is not None:
                    amcl_age = (self.get_clock().now() - self.latest_amcl_time).nanoseconds / 1e9
                    if amcl_age <= self.amcl_pose_stale_sec * 2:  # Allow 2x staleness on timeout
                        self.get_logger().warn(
                            f'AMCL settle: Only received {msgs_received} msgs (wanted {self.amcl_settle_min_msgs}), '
                            f'but accepting with age {amcl_age:.2f}s'
                        )
                        return (True, f'AMCL settled with {msgs_received} messages')
                
                msg = f'AMCL settle timeout after {elapsed:.1f}s (received {msgs_received} msgs, needed {self.amcl_settle_min_msgs})'
                self.get_logger().error(msg)
                return (False, msg)
            
            # Check if we never received any AMCL messages
            if self.latest_amcl_time is None:
                time.sleep(0.1)
                continue
            
            # Check message age
            amcl_age = (self.get_clock().now() - self.latest_amcl_time).nanoseconds / 1e9
            
            # Check if we have enough messages and latest is fresh
            msgs_received = self.amcl_seq - start_seq
            if msgs_received >= self.amcl_settle_min_msgs and amcl_age <= self.amcl_pose_stale_sec:
                self.get_logger().info(f'AMCL settled: received {msgs_received} msgs, age {amcl_age:.2f}s')
                return (True, 'AMCL settled successfully')
            
            time.sleep(0.1)
    
    async def _switch_map(self, floor_id, mode, feedback, goal_handle, initial_pose=None):
        """
        Helper to switch map and set initial pose.
        Args:
            floor_id: Floor identifier
            mode: 'closed' or 'open'
            feedback: Feedback message to update
            goal_handle: Goal handle for publishing feedback
            initial_pose: Optional (x, y, yaw) tuple to publish as initial pose after map load.
                         If None, will auto-retrieve from config based on mode.
        Returns: (success: bool, message: str)
        """
        self.get_logger().info(f'Switching to {mode} map for floor: {floor_id}')
        
        # Determine if we should skip initial pose (open → open transition)
        skip_initial_pose = (self.current_map_mode == 'open' and mode == 'open')
        
        # Get map path
        map_path = self.get_floor_map_yaml(floor_id, mode)
        if map_path is None:
            return (False, f'{mode.capitalize()} map not found for floor "{floor_id}"')
        
        # Update feedback
        feedback.state = f'SWITCHING_TO_{mode.upper()}_MAP'
        feedback.active_floor_id = floor_id
        feedback.active_step = f'LOAD_MAP_{mode.upper()}'
        goal_handle.publish_feedback(feedback)
        
        # Load map
        map_ok, map_msg = await self.load_map(map_path, self.map_switch_timeout)
        if not map_ok:
            return (False, f'Failed to load {mode} map: {map_msg}')
        
        # Update current map mode after successful load
        previous_mode = self.current_map_mode
        self.current_map_mode = mode
        
        # Determine initial pose: use provided pose or auto-retrieve from config
        pose_to_publish = initial_pose
        if pose_to_publish is None:
            pose_to_publish = self.get_amcl_initial_pose(floor_id, mode)
        
        # Publish initial pose AFTER map load (skip for open→open transitions)
        if skip_initial_pose:
            self.get_logger().info(f'Skipping initial pose for {previous_mode}→{mode} map transition (robot inside elevator)')
        elif pose_to_publish is not None:
            x, y, yaw = pose_to_publish
            self.publish_initial_pose(x, y, yaw)
            # Give AMCL a moment to process the initial pose
            time.sleep(0.2)
        else:
            self.get_logger().warn(f'No initial pose available for floor "{floor_id}" mode "{mode}"')
        
        # Clear costmaps if enabled
        if self.clear_costmaps_after_map_switch:
            feedback.state = 'CLEARING_COSTMAPS'
            feedback.active_step = 'CLEAR_COSTMAPS'
            goal_handle.publish_feedback(feedback)
            
            clear_ok, clear_msg = await self.clear_costmaps(self.clear_costmap_timeout)
            if not clear_ok:
                self.get_logger().warn(f'Costmap clear had issues: {clear_msg}')
            else:
                self.get_logger().info('Costmaps cleared successfully')
        
        self.get_logger().info(f'Successfully switched to {mode} map')
        return (True, f'{mode.capitalize()} map loaded')
    
    async def _wait_for_elevator_arrival(self, feedback, goal_handle):
        """
        Wait for elevator door to open (indicating arrival).
        Returns: (success: bool, message: str)
        """
        feedback.state = 'WAIT_FOR_ELEVATOR_ARRIVAL'
        feedback.active_step = 'DOOR_OPEN_ON_TARGET'
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info('Waiting for elevator arrival (door open detection)...')
        if not await self._wait_for_door_open(self.door_open_timeout):
            return (False, 'Elevator did not arrive within timeout (door did not open)')
        
        return (True, 'Elevator arrived (door opened)')
    
    async def _navigate_to_pose(self, goal_handle, target_pose, feedback, label):
        """
        Helper to navigate to a pose using Nav2.
        Returns tuple: (success, nav_status, message)
        """
        # Wait for Nav2 action server
        if not self.wait_for_nav2_server():
            return (False, 0, 'Nav2 action server not available')
        
        # Create Nav2 goal
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = target_pose
        
        self.get_logger().info(f'Sending Nav2 goal to {label}')
        
        # Send goal to Nav2
        send_goal_future = self.nav2_client.send_goal_async(
            nav2_goal,
            feedback_callback=lambda fb: self.nav2_feedback_callback(fb, goal_handle)
        )
        
        # Wait for Nav2 to accept goal
        try:
            nav2_goal_handle = await send_goal_future
        except Exception as e:
            return (False, 0, f'Failed to send goal to Nav2: {str(e)}')
        
        if not nav2_goal_handle.accepted:
            return (False, 0, 'Nav2 rejected the goal')
        
        self.get_logger().info('Nav2 accepted the goal')
        self.active_nav2_goal_handle = nav2_goal_handle
        
        # Wait for Nav2 result
        get_result_future = nav2_goal_handle.get_result_async()
        
        # Periodically check for cancellation while waiting
        feedback_period = 1.0 / self.feedback_rate_hz if self.feedback_rate_hz > 0 else 0.5
        
        while not get_result_future.done():
            # Check if our action was canceled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled by client, canceling Nav2 goal')
                
                # Cancel the Nav2 goal
                cancel_future = nav2_goal_handle.cancel_goal_async()
                await cancel_future
                
                self.active_nav2_goal_handle = None
                return (False, 5, 'Canceled by client')
            
            # Send periodic feedback
            feedback.active_step = 'in_progress'
            goal_handle.publish_feedback(feedback)
            
            # Sleep briefly - use time.sleep() as ROS 2 doesn't provide asyncio event loop
            time.sleep(feedback_period)
        
        # Get Nav2 result
        nav2_result = await get_result_future
        self.active_nav2_goal_handle = None
        
        status = nav2_result.status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Successfully navigated to {label}')
            return (True, status, f'Reached {label}')
        elif status == 5:  # CANCELED
            return (False, status, f'Navigation to {label} was canceled')
        elif status == 6:  # ABORTED
            return (False, status, f'Navigation to {label} was aborted')
        else:
            return (False, status, f'Navigation completed with status {status}')
    
    async def _do_elevator_entry_sequence(self, goal_handle, request, feedback):
        """
        Execute full elevator entry sequence and navigate to final target.
        Args:
            goal_handle: Action goal handle
            request: NavigateToNamedLocation.Goal (contains target_floor_id and final x, y, yaw)
            feedback: Feedback message to update
        Returns: (success: bool, message: str)
        """
        target_floor_id = request.target_floor_id
        
        # Step 1: Call elevator to current floor
        feedback.state = 'ELEVATOR_CALLING_CURRENT_FLOOR'
        feedback.active_step = 'CALL_ELEVATOR'
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info(f'Calling elevator to current floor: {self.current_floor_id}')
        if not self._send_elevator_to_floor(self.current_floor_id):
            self.stop_robot()
            return (False, f'Failed to call elevator to current floor {self.current_floor_id}')
        
        # Step 2: Wait for door to open
        feedback.state = 'WAITING_FOR_DOOR_OPEN'
        feedback.active_step = 'DOOR_OPEN_DETECTION'
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info('Waiting for elevator door to open...')
        if not await self._wait_for_door_open(self.door_open_timeout):
            self.stop_robot()
            return (False, 'Elevator door did not open within timeout')
        
        # Step 3: Navigate to elevator_inside pose
        feedback.state = 'ENTERING_ELEVATOR'
        feedback.active_step = 'NAVIGATE_TO_INSIDE'
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info('Door open detected, navigating to elevator_inside...')
        
        # Get elevator_inside pose for current floor
        inside_pose = self.get_named_pose(self.current_floor_id, 'elevator_inside')
        if inside_pose is None:
            self.stop_robot()
            return (False, f'elevator_inside not found on current floor "{self.current_floor_id}"')
        
        ix, iy, iyaw = inside_pose
        self.get_logger().info(
            f'Navigating to elevator_inside on {self.current_floor_id}: ({ix:.2f}, {iy:.2f}, {math.degrees(iyaw):.1f}°)'
        )
        
        # Build PoseStamped for inside position
        inside_pose_stamped = self.create_pose_stamped(ix, iy, iyaw)
        
        # Navigate to inside pose
        nav_result = await self._navigate_to_pose(goal_handle, inside_pose_stamped, feedback, 'elevator_inside')
        
        if not nav_result[0]:
            self.stop_robot()
            return (False, f'Failed to enter elevator: {nav_result[2]}')
        
        # Step 4: Call elevator to target floor
        feedback.state = 'ELEVATOR_CALLING_TARGET_FLOOR'
        feedback.active_step = 'CALL_TARGET_FLOOR'
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info(f'Calling elevator to target floor: {target_floor_id}')
        if not self._send_elevator_to_floor(target_floor_id):
            self.stop_robot()
            return (False, f'Failed to call elevator to target floor {target_floor_id}')
        
        # Step 5: Switch to target floor's open map and wait for AMCL settle
        self.get_logger().info(f'Switching to target floor open map: {target_floor_id}')
        
        switch_ok, switch_msg = await self._switch_map(target_floor_id, 'open', feedback, goal_handle)
        if not switch_ok:
            self.stop_robot()
            return (False, switch_msg)
        
        # Step 6: Wait for elevator arrival on target floor
        arrival_ok, arrival_msg = await self._wait_for_elevator_arrival(feedback, goal_handle)
        if not arrival_ok:
            self.stop_robot()
            return (False, arrival_msg)
        
        # Step 7: Navigate to elevator_exit on target floor
        feedback.state = 'EXITING_ELEVATOR'
        feedback.active_floor_id = target_floor_id
        feedback.active_step = 'NAVIGATE_TO_EXIT'
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info(f'Elevator arrived, navigating to elevator_exit...')
        
        # Get elevator_exit pose for target floor
        exit_pose = self.get_named_pose(target_floor_id, 'elevator_exit')
        if exit_pose is None:
            self.stop_robot()
            return (False, f'elevator_exit not found on target floor "{target_floor_id}"')
        
        ex, ey, eyaw = exit_pose
        self.get_logger().info(
            f'Navigating to elevator_exit on {target_floor_id}: ({ex:.2f}, {ey:.2f}, {math.degrees(eyaw):.1f}°)'
        )
        
        # Build PoseStamped for exit position
        exit_pose_stamped = self.create_pose_stamped(ex, ey, eyaw)
        
        # Navigate to exit pose
        nav_result = await self._navigate_to_pose(goal_handle, exit_pose_stamped, feedback, 'elevator_exit')
        
        if not nav_result[0]:
            self.stop_robot()
            return (False, f'Failed to exit elevator: {nav_result[2]}')
        
        # Step 8: Switch back to target floor's closed map with auto initial pose
        self.get_logger().info(f'Robot at elevator_exit, switching to closed map for floor: {target_floor_id}')
        
        # _switch_map will auto-retrieve amcl_initial_pose_closed from config
        switch_ok, switch_msg = await self._switch_map(
            target_floor_id, 'closed', feedback, goal_handle
        )
        if not switch_ok:
            self.stop_robot()
            return (False, switch_msg)
        
        # Initial pose published - AMCL should localize quickly with calibrated pose
        # Skip settle check since robot is stationary and AMCL may not publish until movement
        self.get_logger().info('Initial pose set on closed map, proceeding to final navigation')
        
        # Step 9: Update current floor to target floor NOW (after all map switches succeeded)
        self.current_floor_id = target_floor_id
        self.publish_current_floor()
        self.get_logger().info(f'Current floor updated to: {self.current_floor_id}')
        
        # Step 10: Navigate to final target pose
        feedback.state = 'FINAL_NAV_ON_TARGET_FLOOR'
        feedback.active_floor_id = target_floor_id
        feedback.active_step = 'NAVIGATE_TO_FINAL_TARGET'
        goal_handle.publish_feedback(feedback)
        
        self.get_logger().info(
            f'Navigating to final target: ({request.x:.2f}, {request.y:.2f}, {math.degrees(request.yaw):.1f}°)'
        )
        
        # Build PoseStamped from original request
        final_pose_stamped = self.create_pose_stamped(request.x, request.y, request.yaw)
        
        # Navigate to final target
        nav_result = await self._navigate_to_pose(goal_handle, final_pose_stamped, feedback, 'final_target')
        
        if not nav_result[0]:
            self.stop_robot()
            return (False, f'Failed to reach final target: {nav_result[2]}')
        
        # Success - completed full cross-floor navigation
        message = (
            f'Successfully completed cross-floor navigation to {target_floor_id}. '
            f'Robot reached final target ({request.x:.2f}, {request.y:.2f}).'
        )
        self.get_logger().info(message)
        return (True, message)
    
    def _send_elevator_to_floor(self, floor_id):
        """
        Send elevator to specified floor using Gazebo CLI.
        Returns: bool (success)
        """
        # Convert floor_id to elevator floor number
        floor_map = {
            'floor0': '0',
            'floor1': '1',
            'floor2': '2',
            'floor3': '3'
        }
        
        if floor_id not in floor_map:
            self.get_logger().error(f'Unknown floor_id: {floor_id}')
            return False
        
        floor_num = floor_map[floor_id]
        
        try:
            cmd = [
                self.gz_cli,
                'topic',
                '-p',
                self.gz_elevator_topic,
                '-m',
                f'data: "{floor_num}"'
            ]
            
            self.get_logger().info(f'Executing: {" ".join(cmd)}')
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            
            if result.returncode == 0:
                self.get_logger().info(f'Elevator commanded to floor {floor_num}')
                return True
            else:
                self.get_logger().error(
                    f'Elevator command failed with code {result.returncode}\\n'
                    f'stdout: {result.stdout}\\nstderr: {result.stderr}'
                )
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Elevator command timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Exception calling elevator: {e}')
            return False
    
    def scan_callback(self, msg):
        """Store latest LaserScan for door detection."""
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()
    
    def odom_callback(self, msg):
        """Store latest Odometry for guarded motion."""
        self.latest_odom = msg
        self.latest_odom_time = self.get_clock().now()
    
    def amcl_callback(self, msg):
        """Store latest AMCL pose for settle check."""
        self.latest_amcl_pose = msg
        self.latest_amcl_time = self.get_clock().now()
        self.amcl_seq += 1
    
    def is_door_open_from_scan(self, scan):
        """
        Check if door is open based on LaserScan data.
        Returns: bool
        """
        if scan is None:
            return False
        
        # Convert window parameters to radians
        center_rad = math.radians(self.door_window_center_deg)
        width_rad = math.radians(self.door_window_width_deg)
        
        window_min = center_rad - width_rad / 2.0
        window_max = center_rad + width_rad / 2.0
        
        # Compute indices covering the window
        # angle_i = angle_min + i * angle_increment
        # Solve for i when angle_i is in [window_min, window_max]
        
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_inc = scan.angle_increment
        num_rays = len(scan.ranges)
        
        # Clamp window to scan range
        window_min_clamped = max(window_min, angle_min)
        window_max_clamped = min(window_max, angle_max)
        
        if window_min_clamped >= window_max_clamped:
            self.get_logger().warn('Door detection window outside scan range')
            return False
        
        # Compute index range
        idx_min = int((window_min_clamped - angle_min) / angle_inc)
        idx_max = int((window_max_clamped - angle_min) / angle_inc)
        
        idx_min = max(0, idx_min)
        idx_max = min(num_rays - 1, idx_max)
        
        if idx_min >= idx_max:
            return False
        
        # Count valid rays and open rays
        valid_count = 0
        open_count = 0
        
        for i in range(idx_min, idx_max + 1):
            r = scan.ranges[i]
            
            # Skip invalid ranges
            if not math.isfinite(r):
                continue
            
            # Clamp to valid range
            r = max(scan.range_min, min(scan.range_max, r))
            
            valid_count += 1
            if r > self.door_open_range_threshold:
                open_count += 1
        
        if valid_count < 5:
            return False
        
        fraction = open_count / valid_count
        return fraction >= self.door_open_fraction_threshold
    
    async def _wait_for_door_open(self, timeout_sec):
        """
        Wait for door to open with stable detection.
        Returns: bool (success)
        """
        start_time = self.get_clock().now()
        door_open_start_time = None
        
        while True:
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            if elapsed > timeout_sec:
                self.get_logger().error(f'Door open timeout after {timeout_sec:.1f}s')
                return False
            
            # Check if scan is recent
            if self.latest_scan_time is None:
                self.get_logger().warn('No scan received yet')
                time.sleep(1.0 / self.door_poll_rate_hz)
                continue
            
            scan_age = (self.get_clock().now() - self.latest_scan_time).nanoseconds / 1e9
            if scan_age > 1.0:
                self.get_logger().error(f'Scan is stale (age: {scan_age:.2f}s)')
                return False
            
            # Check door state
            door_open = self.is_door_open_from_scan(self.latest_scan)
            
            if door_open:
                if door_open_start_time is None:
                    door_open_start_time = self.get_clock().now()
                    self.get_logger().info('Door open detected, waiting for stability...')
                else:
                    stable_duration = (self.get_clock().now() - door_open_start_time).nanoseconds / 1e9
                    if stable_duration >= self.door_open_stable_time:
                        self.get_logger().info(f'Door open stable for {stable_duration:.2f}s')
                        return True
            else:
                if door_open_start_time is not None:
                    self.get_logger().warn('Door open condition lost, resetting timer')
                door_open_start_time = None
            
            time.sleep(1.0 / self.door_poll_rate_hz)
    
    def get_min_front_clearance_from_scan(self, scan):
        """
        Get minimum clearance in front check window.
        Returns: float (min range, or +inf if no valid rays)
        """
        if scan is None:
            return float('inf')
        
        # Convert window parameters to radians
        center_rad = 0.0  # Forward direction
        width_rad = math.radians(self.front_check_window_deg)
        
        window_min = center_rad - width_rad / 2.0
        window_max = center_rad + width_rad / 2.0
        
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_inc = scan.angle_increment
        num_rays = len(scan.ranges)
        
        # Clamp window to scan range
        window_min_clamped = max(window_min, angle_min)
        window_max_clamped = min(window_max, angle_max)
        
        if window_min_clamped >= window_max_clamped:
            self.get_logger().warn('Front check window outside scan range')
            return float('inf')
        
        # Compute index range
        idx_min = int((window_min_clamped - angle_min) / angle_inc)
        idx_max = int((window_max_clamped - angle_min) / angle_inc)
        
        idx_min = max(0, idx_min)
        idx_max = min(num_rays - 1, idx_max)
        
        min_range = float('inf')
        
        for i in range(idx_min, idx_max + 1):
            r = scan.ranges[i]
            
            if not math.isfinite(r):
                continue
            
            r = max(scan.range_min, min(scan.range_max, r))
            min_range = min(min_range, r)
        
        if min_range == float('inf'):
            self.get_logger().warn('No valid ranges in front check window')
        
        return min_range
    
    def publish_cmd_vel(self, linear_x, angular_z=0.0):
        """Publish velocity command."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)
    
    def stop_robot(self):
        """Stop robot by publishing zero velocity multiple times."""
        for _ in range(3):
            self.publish_cmd_vel(0.0, 0.0)
            time.sleep(0.05)
    
    async def _drive_forward_guarded(self, distance, speed, timeout_sec):
        """
        Drive forward with obstacle checking.
        Returns: bool (success)
        """
        start_time = self.get_clock().now()
        
        # Try to use odometry for distance tracking
        start_odom = self.latest_odom
        use_odom = start_odom is not None
        
        if use_odom:
            start_x = start_odom.pose.pose.position.x
            start_y = start_odom.pose.pose.position.y
            self.get_logger().info(f'Using odometry for distance tracking (start: {start_x:.2f}, {start_y:.2f})')
        else:
            self.get_logger().warn('No odometry available, using time-based distance estimate')
        
        traveled = 0.0
        
        while traveled < distance:
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            if elapsed > timeout_sec:
                self.get_logger().error(f'Guarded motion timeout after {elapsed:.1f}s')
                self.stop_robot()
                return False
            
            # Check scan age
            if self.latest_scan_time is None:
                self.get_logger().error('No scan available for obstacle checking')
                self.stop_robot()
                return False
            
            scan_age = (self.get_clock().now() - self.latest_scan_time).nanoseconds / 1e9
            if scan_age > 1.0:
                self.get_logger().error(f'Scan is stale (age: {scan_age:.2f}s)')
                self.stop_robot()
                return False
            
            # Check front clearance
            clearance = self.get_min_front_clearance_from_scan(self.latest_scan)
            if clearance < self.min_front_clearance:
                self.get_logger().error(f'Obstacle detected! Clearance: {clearance:.2f}m < {self.min_front_clearance:.2f}m')
                self.stop_robot()
                return False
            
            # Publish forward velocity
            self.publish_cmd_vel(speed)
            
            # Update traveled distance
            if use_odom and self.latest_odom is not None:
                current_x = self.latest_odom.pose.pose.position.x
                current_y = self.latest_odom.pose.pose.position.y
                dx = current_x - start_x
                dy = current_y - start_y
                traveled = math.sqrt(dx*dx + dy*dy)
            else:
                traveled = elapsed * speed
            
            time.sleep(1.0 / self.motion_control_rate)
        
        self.get_logger().info(f'Successfully traveled {traveled:.2f}m')
        self.stop_robot()
        return True
    
    def nav2_feedback_callback(self, feedback_msg, goal_handle):
        """
        Receive feedback from Nav2 (optional processing).
        We already send periodic feedback in execute_callback.
        """
        # Optionally log or process Nav2 feedback
        # For now, we handle feedback in the main execution loop
        pass


def main(args=None):
    rclpy.init(args=args)
    
    node = SMRRMultiFloorBTNavigator()
    
    # Use MultiThreadedExecutor for concurrent action handling
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception in smrr_multifloor_bt_navigator: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
