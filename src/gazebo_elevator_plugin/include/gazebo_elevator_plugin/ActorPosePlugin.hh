#ifndef GAZEBO_ACTOR_POSE_PLUGIN_HH
#define GAZEBO_ACTOR_POSE_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace gazebo
{
  /// \brief Custom message for actor state (pose + velocity + name)
  struct ActorState
  {
    std::string name;
    ignition::math::Pose3d pose;
    ignition::math::Vector3d linear_velocity;
    ignition::math::Vector3d angular_velocity;
  };

  /// \brief A plugin that publishes ground truth poses and velocities of all actors in the world
  class ActorPosePlugin : public WorldPlugin
  {
    public:
      /// \brief Constructor
      ActorPosePlugin() : WorldPlugin()
      {
      }

      /// \brief Destructor
      virtual ~ActorPosePlugin()
      {
        this->update_connection_.reset();
        if (rclcpp::ok())
        {
          this->ros_node_.reset();
        }
      }

      /// \brief Load the plugin
      void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
      {
        this->world_ = _world;

        // Initialize ROS 2 if not already initialized
        if (!rclcpp::ok())
        {
          rclcpp::init(0, nullptr);
        }

        // Create ROS 2 node
        this->ros_node_ = std::make_shared<rclcpp::Node>("actor_ground_truth_publisher");

        // Create publishers
        this->pose_pub_ = this->ros_node_->create_publisher<geometry_msgs::msg::PoseArray>(
            "/actors/ground_truth/poses", 10);
        this->marker_pub_ = this->ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/actors/ground_truth/markers", 10);
        
        // Publisher for individual actor odometry (pose + velocity)
        // Will create dynamically for each actor

        // Get update rate from SDF (default: 10 Hz)
        this->update_rate_ = 10.0;
        if (_sdf->HasElement("update_rate"))
        {
          this->update_rate_ = _sdf->Get<double>("update_rate");
        }
        this->update_period_ = 1.0 / this->update_rate_;

        // Connect to world update event
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ActorPosePlugin::OnUpdate, this));

        this->last_update_time_ = this->world_->SimTime();

        RCLCPP_INFO(this->ros_node_->get_logger(), 
                    "Actor Ground Truth Plugin loaded. Publishing at %.1f Hz", this->update_rate_);
        RCLCPP_INFO(this->ros_node_->get_logger(),
                    "Coordinate transform: Gazebo world (0,0) = map frame (-2, 2)");
      }

    private:
      /// \brief Called every simulation iteration
      void OnUpdate()
      {
        // Rate limiting
        common::Time current_time = this->world_->SimTime();
        double dt = (current_time - this->last_update_time_).Double();
        
        if (dt < this->update_period_)
        {
          return;
        }
        this->last_update_time_ = current_time;

        // Spin ROS 2 node
        rclcpp::spin_some(this->ros_node_);

        // Get all actors directly using GetName and GetByName
        std::vector<ActorState> actor_states;
        
        // Iterate through all models and check for actors
        physics::Model_V models = this->world_->Models();
        for (auto model : models)
        {
          // Try to cast to Actor
          physics::ActorPtr actor = boost::dynamic_pointer_cast<physics::Actor>(model);
          
          if (actor)
          {
            ActorState state;
            state.name = actor->GetName();
            
            // Get pose directly from actor - this is the ground truth from Gazebo
            ignition::math::Pose3d world_pose = actor->WorldPose();
            
            // Transform from Gazebo world coordinates to map frame
            // Gazebo world (0,0) = map frame (-2, 2)
            // map_x = world_x - 2
            // map_y = world_y + 2
            ignition::math::Pose3d map_pose = world_pose;
            map_pose.Pos().X(world_pose.Pos().X() - 2.0);
            map_pose.Pos().Y(world_pose.Pos().Y() + 2.0);
            
            state.pose = map_pose;
            
            // Calculate velocity from pose change
            if (this->last_actor_poses_.find(state.name) != this->last_actor_poses_.end())
            {
              ignition::math::Pose3d last_pose = this->last_actor_poses_[state.name];
              double actual_dt = dt > 0.001 ? dt : 0.1;
              
              state.linear_velocity.X((map_pose.Pos().X() - last_pose.Pos().X()) / actual_dt);
              state.linear_velocity.Y((map_pose.Pos().Y() - last_pose.Pos().Y()) / actual_dt);
              state.linear_velocity.Z((map_pose.Pos().Z() - last_pose.Pos().Z()) / actual_dt);
            }
            else
            {
              state.linear_velocity.X(0.0);
              state.linear_velocity.Y(0.0);
              state.linear_velocity.Z(0.0);
            }
            
            state.angular_velocity.X(0.0);
            state.angular_velocity.Y(0.0);
            state.angular_velocity.Z(0.0);
            
            // Store for next iteration (in map frame)
            this->last_actor_poses_[state.name] = map_pose;
            
            actor_states.push_back(state);
            
            // Debug output
            if (this->debug_counter_ % 50 == 0)  // Every 5 seconds at 10Hz
            {
              RCLCPP_INFO(this->ros_node_->get_logger(),
                         "Actor %s: Gazebo(%.2f, %.2f) -> map(%.2f, %.2f) vel=(%.2f, %.2f)",
                         state.name.c_str(),
                         world_pose.Pos().X(), world_pose.Pos().Y(),
                         map_pose.Pos().X(), map_pose.Pos().Y(),
                         state.linear_velocity.X(), state.linear_velocity.Y());
            }
          }
        }
        
        this->debug_counter_++;

        if (actor_states.empty())
        {
          if (this->debug_counter_ % 100 == 0)
          {
            RCLCPP_WARN(this->ros_node_->get_logger(), "No actors found in simulation!");
          }
          return;
        }

        // Publish poses
        this->PublishPoses(actor_states, current_time);

        // Publish odometry for each actor
        this->PublishOdometry(actor_states, current_time);

        // Publish visualization markers
        this->PublishMarkers(actor_states, current_time);
      }

      /// \brief Publish actor poses
      void PublishPoses(const std::vector<ActorState>& states, const common::Time& time)
      {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp.sec = time.sec;
        pose_array.header.stamp.nanosec = time.nsec;
        pose_array.header.frame_id = "map";  // Transformed to map frame

        for (const auto& state : states)
        {
          geometry_msgs::msg::Pose pose;
          pose.position.x = state.pose.Pos().X();
          pose.position.y = state.pose.Pos().Y();
          pose.position.z = state.pose.Pos().Z();
          pose.orientation.x = state.pose.Rot().X();
          pose.orientation.y = state.pose.Rot().Y();
          pose.orientation.z = state.pose.Rot().Z();
          pose.orientation.w = state.pose.Rot().W();
          pose_array.poses.push_back(pose);
        }

        this->pose_pub_->publish(pose_array);
      }

      /// \brief Publish actor odometry (pose + velocity)
      void PublishOdometry(const std::vector<ActorState>& states, const common::Time& time)
      {
        for (const auto& state : states)
        {
          // Create publisher for this actor if it doesn't exist
          if (this->odom_pubs_.find(state.name) == this->odom_pubs_.end())
          {
            std::string topic = "/actors/ground_truth/" + state.name + "/odom";
            this->odom_pubs_[state.name] = 
                this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(topic, 10);
            RCLCPP_INFO(this->ros_node_->get_logger(), 
                        "Created odometry publisher for %s at %s", state.name.c_str(), topic.c_str());
          }

          // Publish odometry message
          nav_msgs::msg::Odometry odom;
          odom.header.stamp.sec = time.sec;
          odom.header.stamp.nanosec = time.nsec;
          odom.header.frame_id = "map";  // Transformed to map frame
          odom.child_frame_id = state.name;

          // Pose
          odom.pose.pose.position.x = state.pose.Pos().X();
          odom.pose.pose.position.y = state.pose.Pos().Y();
          odom.pose.pose.position.z = state.pose.Pos().Z();
          odom.pose.pose.orientation.x = state.pose.Rot().X();
          odom.pose.pose.orientation.y = state.pose.Rot().Y();
          odom.pose.pose.orientation.z = state.pose.Rot().Z();
          odom.pose.pose.orientation.w = state.pose.Rot().W();

          // Velocity
          odom.twist.twist.linear.x = state.linear_velocity.X();
          odom.twist.twist.linear.y = state.linear_velocity.Y();
          odom.twist.twist.linear.z = state.linear_velocity.Z();
          odom.twist.twist.angular.x = state.angular_velocity.X();
          odom.twist.twist.angular.y = state.angular_velocity.Y();
          odom.twist.twist.angular.z = state.angular_velocity.Z();

          this->odom_pubs_[state.name]->publish(odom);
        }
      }

      /// \brief Publish actor velocities (DEPRECATED - use odometry instead)
      void PublishVelocities(const std::vector<ActorState>& states, const common::Time& time)
      {
        // This function is deprecated - velocities are now published in odometry messages
      }

      /// \brief Publish visualization markers
      void PublishMarkers(const std::vector<ActorState>& states, const common::Time& time)
      {
        visualization_msgs::msg::MarkerArray marker_array;

        int marker_id = 0;
        for (const auto& state : states)
        {
          // Actor position marker (sphere)
          visualization_msgs::msg::Marker marker;
          marker.header.stamp.sec = time.sec;
          marker.header.stamp.nanosec = time.nsec;
          marker.header.frame_id = "map";  // Transformed to map frame
          marker.ns = "actor_ground_truth";
          marker.id = marker_id++;
          marker.type = visualization_msgs::msg::Marker::CYLINDER;
          marker.action = visualization_msgs::msg::Marker::ADD;
          
          marker.pose.position.x = state.pose.Pos().X();
          marker.pose.position.y = state.pose.Pos().Y();
          marker.pose.position.z = 0.9; // Cylinder center height
          marker.pose.orientation.w = 1.0;
          
          marker.scale.x = 0.5;
          marker.scale.y = 0.5;
          marker.scale.z = 1.8;
          
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 1.0;
          marker.color.a = 0.6;
          
          marker.lifetime.sec = 0;
          marker.lifetime.nanosec = 200000000; // 0.2 seconds
          
          marker_array.markers.push_back(marker);

          // Velocity arrow
          double vel_mag = state.linear_velocity.Length();
          if (vel_mag > 0.01)
          {
            visualization_msgs::msg::Marker arrow;
            arrow.header.stamp.sec = time.sec;
            arrow.header.stamp.nanosec = time.nsec;
            arrow.header.frame_id = "map";  // Transformed to map frame
            arrow.ns = "actor_velocities";
            arrow.id = marker_id++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            
            geometry_msgs::msg::Point start, end;
            start.x = state.pose.Pos().X();
            start.y = state.pose.Pos().Y();
            start.z = 0.1;
            
            end.x = start.x + state.linear_velocity.X();
            end.y = start.y + state.linear_velocity.Y();
            end.z = 0.1;
            
            arrow.points.push_back(start);
            arrow.points.push_back(end);
            
            arrow.scale.x = 0.1;
            arrow.scale.y = 0.15;
            arrow.scale.z = 0.2;
            
            arrow.color.r = 0.0;
            arrow.color.g = 1.0;
            arrow.color.b = 1.0;
            arrow.color.a = 1.0;
            
            arrow.lifetime.sec = 0;
            arrow.lifetime.nanosec = 200000000;
            
            marker_array.markers.push_back(arrow);
          }

          // Text label with name and velocity
          visualization_msgs::msg::Marker text;
          text.header.stamp.sec = time.sec;
          text.header.stamp.nanosec = time.nsec;
          text.header.frame_id = "map";  // Transformed to map frame
          text.ns = "actor_labels";
          text.id = marker_id++;
          text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          text.action = visualization_msgs::msg::Marker::ADD;
          
          text.pose.position.x = state.pose.Pos().X();
          text.pose.position.y = state.pose.Pos().Y();
          text.pose.position.z = 2.0;
          text.pose.orientation.w = 1.0;
          
          double speed = state.linear_velocity.Length();
          text.text = state.name + "\n" + 
                      std::to_string(speed).substr(0, 4) + " m/s";
          text.scale.z = 0.3;
          
          text.color.r = 0.0;
          text.color.g = 1.0;
          text.color.b = 1.0;
          text.color.a = 1.0;
          
          text.lifetime.sec = 0;
          text.lifetime.nanosec = 200000000;
          
          marker_array.markers.push_back(text);
        }

        this->marker_pub_->publish(marker_array);
      }

      /// \brief Pointer to the world
      physics::WorldPtr world_;

      /// \brief Connection to world update event
      event::ConnectionPtr update_connection_;

      /// \brief ROS 2 node
      std::shared_ptr<rclcpp::Node> ros_node_;

      /// \brief Publishers
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
      std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;

      /// \brief Update rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;
      
      /// \brief Storage for velocity calculation
      std::map<std::string, ignition::math::Pose3d> last_actor_poses_;
      
      /// \brief Debug counter
      int debug_counter_ = 0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ActorPosePlugin)
}

#endif
