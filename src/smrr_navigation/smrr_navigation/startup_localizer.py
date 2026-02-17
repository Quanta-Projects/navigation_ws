#!/usr/bin/env python3
"""
Startup Localizer Node for AMCL Convergence

This node runs automatically at navigation startup to help AMCL localization converge.
It performs a simple motion sequence:
1. Waits for initialization
2. Drives forward ~1 meter (odometry-based)
3. Performs a 360-degree rotation in place (yaw-based)
4. Stops and exits

This motion helps AMCL quickly refine the initial pose estimate by observing
the environment from multiple perspectives.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import tf2_ros
from tf2_ros import TransformException


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(quat):
    """Convert quaternion to yaw angle"""
    # quat = (x, y, z, w)
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


class StartupLocalizer(Node):
    """
    A simple node that drives the robot through a predefined motion sequence
    to help AMCL localization converge at startup.
    Uses odometry feedback for distance and rotation control.
    """
    
    def __init__(self):
        super().__init__('startup_localizer')
        
        # Declare parameters for easy tuning
        self.declare_parameter('startup_delay', 2.0)  # Wait for AMCL to initialize
        self.declare_parameter('forward_speed', 0.15)  # m/s
        self.declare_parameter('forward_distance', 1.0)  # meters
        self.declare_parameter('rotation_speed', 0.5)  # rad/s
        self.declare_parameter('target_rotation_angle', 2.0 * math.pi)  # radians (360°)
        self.declare_parameter('control_period', 0.05)  # timer period in seconds
        self.declare_parameter('stop_duration', 1.0)  # Pause between motions
        
        # Get parameters
        self.startup_delay = self.get_parameter('startup_delay').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.forward_distance = self.get_parameter('forward_distance').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.target_rotation = self.get_parameter('target_rotation_angle').value
        self.control_period = self.get_parameter('control_period').value
        self.stop_dur = self.get_parameter('stop_duration').value
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )
        
        # Optional: Subscribe to AMCL pose to monitor convergence
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        
        # TF listener (alternative to odometry subscription)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State machine variables
        self.state = 'INIT'
        self.state_start_time = None
        self.amcl_pose = None
        self.current_odom = None
        
        # Motion tracking variables
        self.forward_start_x = None
        self.forward_start_y = None
        self.rotation_start_yaw = None
        self.accumulated_rotation = 0.0
        self.last_yaw = None
        
        # Create timer for state machine
        self.timer = self.create_timer(self.control_period, self.timer_callback)
        
        self.get_logger().info('Startup Localizer initialized (odometry-based)')
        self.get_logger().info(f'Sequence: Wait {self.startup_delay}s -> '
                              f'Drive {self.forward_distance}m @ {self.forward_speed}m/s -> '
                              f'Stop {self.stop_dur}s -> '
                              f'Rotate {math.degrees(self.target_rotation):.1f}° @ {self.rotation_speed}rad/s')
    
    def odom_callback(self, msg):
        """Store current odometry data"""
        self.current_odom = msg
    
    def amcl_pose_callback(self, msg):
        """
        Optional callback to track AMCL pose.
        Can be extended to check covariance and determine convergence.
        """
        self.amcl_pose = msg
    
    def get_current_pose_from_odom(self):
        """
        Get current pose from odometry.
        Returns (x, y, yaw) or None if not available.
        """
        if self.current_odom is None:
            return None
        
        pose = self.current_odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(pose.orientation)
        
        return (x, y, yaw)
    
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Helper function to publish velocity commands"""
        if not rclpy.ok() or self.cmd_vel_pub is None:
            return
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
    
    def get_elapsed_time(self):
        """Get time elapsed in current state"""
        if self.state_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
    
    def change_state(self, new_state):
        """Change to a new state and log the transition"""
        self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        
        # Reset tracking variables when entering new motion states
        if new_state == 'FORWARD':
            pose = self.get_current_pose_from_odom()
            if pose:
                self.forward_start_x = pose[0]
                self.forward_start_y = pose[1]
                self.get_logger().info(f'Forward motion starting from ({pose[0]:.3f}, {pose[1]:.3f})')
            else:
                self.get_logger().warn('No odometry data available at forward start')
        
        elif new_state == 'ROTATE':
            pose = self.get_current_pose_from_odom()
            if pose:
                self.rotation_start_yaw = pose[2]
                self.last_yaw = pose[2]
                self.accumulated_rotation = 0.0
                self.get_logger().info(f'Rotation starting at yaw {math.degrees(pose[2]):.1f}°')
            else:
                self.get_logger().warn('No odometry data available at rotation start')
    
    def timer_callback(self):
        """
        State machine timer callback.
        States: INIT -> FORWARD -> STOP1 -> ROTATE -> STOP2 -> DONE
        """
        elapsed = self.get_elapsed_time()
        
        if self.state == 'INIT':
            # Initial delay to let AMCL initialize
            self.publish_velocity(0.0, 0.0)
            if self.state_start_time is None:
                self.state_start_time = self.get_clock().now()
            elif elapsed >= self.startup_delay:
                self.change_state('FORWARD')
        
        elif self.state == 'FORWARD':
            # Drive forward until target distance reached (odometry-based)
            pose = self.get_current_pose_from_odom()
            
            if pose is None:
                self.get_logger().warn('Waiting for odometry data...', throttle_duration_sec=1.0)
                self.publish_velocity(0.0, 0.0)
                return
            
            if self.forward_start_x is None or self.forward_start_y is None:
                # Initialize starting position if not set
                self.forward_start_x = pose[0]
                self.forward_start_y = pose[1]
                self.get_logger().info(f'Forward motion initialized at ({pose[0]:.3f}, {pose[1]:.3f})')
            
            # Calculate distance travelled
            dx = pose[0] - self.forward_start_x
            dy = pose[1] - self.forward_start_y
            dist = math.sqrt(dx * dx + dy * dy)
            
            if dist < self.forward_distance:
                # Keep driving forward
                self.publish_velocity(self.forward_speed, 0.0)
                if int(elapsed * 2) % 2 == 0:  # Log every 0.5 seconds
                    self.get_logger().info(f'Forward: {dist:.3f}m / {self.forward_distance}m', 
                                          throttle_duration_sec=0.5)
            else:
                # Target distance reached
                self.get_logger().info(f'Forward motion complete: travelled {dist:.3f}m')
                self.publish_velocity(0.0, 0.0)
                self.change_state('STOP1')
        
        elif self.state == 'STOP1':
            # Brief stop between forward and rotation
            self.publish_velocity(0.0, 0.0)
            if elapsed >= self.stop_dur:
                self.change_state('ROTATE')
        
        elif self.state == 'ROTATE':
            # Perform rotation based on accumulated yaw change
            pose = self.get_current_pose_from_odom()
            
            if pose is None:
                self.get_logger().warn('Waiting for odometry data...', throttle_duration_sec=1.0)
                self.publish_velocity(0.0, 0.0)
                return
            
            current_yaw = pose[2]
            
            if self.last_yaw is not None:
                # Calculate yaw change since last callback
                yaw_diff = normalize_angle(current_yaw - self.last_yaw)
                self.accumulated_rotation += abs(yaw_diff)
            
            self.last_yaw = current_yaw
            
            if self.accumulated_rotation < self.target_rotation:
                # Keep rotating
                self.publish_velocity(0.0, self.rotation_speed)
                if int(elapsed * 2) % 2 == 0:  # Log every 0.5 seconds
                    progress = (self.accumulated_rotation / self.target_rotation) * 100.0
                    self.get_logger().info(f'Rotation: {math.degrees(self.accumulated_rotation):.1f}° / '
                                          f'{math.degrees(self.target_rotation):.1f}° ({progress:.1f}%)', 
                                          throttle_duration_sec=0.5)
            else:
                # Target rotation reached
                self.get_logger().info(f'Rotation complete: {math.degrees(self.accumulated_rotation):.1f}°')
                self.publish_velocity(0.0, 0.0)
                self.change_state('STOP2')
        
        elif self.state == 'STOP2':
            # Final stop
            self.publish_velocity(0.0, 0.0)
            if elapsed >= self.stop_dur:
                self.change_state('DONE')
        
        elif self.state == 'DONE':
            # Ensure stopped and shut down
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('Startup localization sequence completed!')
            self.get_logger().info('AMCL should now be converged. Node shutting down.')
            
            # Publish stop one more time to be sure
            self.publish_velocity(0.0, 0.0)
            
            # Shutdown after a brief delay
            self.timer.cancel()
            self.create_timer(1.0, self.shutdown_node)
    
    def shutdown_node(self):
        """Shutdown the node gracefully"""
        self.get_logger().info('Shutting down startup_localizer node')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = StartupLocalizer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception in startup_localizer: {e}')
    finally:
        # Ensure robot is stopped before shutdown
        if rclpy.ok():
            node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
