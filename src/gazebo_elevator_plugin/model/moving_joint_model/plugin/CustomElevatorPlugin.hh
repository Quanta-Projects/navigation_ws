#ifndef GAZEBO_PLUGINS_CUSTOMELEVATORPLUGIN_HH_
#define GAZEBO_PLUGINS_CUSTOMELEVATORPLUGIN_HH_

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  /// Forward declare private data.
  class CustomElevatorPluginPrivate;

  /// \brief Plugin to control a elevator. This plugin will listen for
  /// door and lift events on a specified topic.
  ///
  /// \verbatim
  ///   <plugin filename="libCustomElevatorPlugin.so" name="elevator_plugin">
  ///     <lift_joint>elevator::lift</lift_joint>
  ///     <door_joint>elevator::door</door_joint>
  ///     <floor_height>3.075</floor_height>
  ///
  ///     <!-- Time the elevator door will stay open in seconds -->
  ///     <door_wait_time>5</door_wait_time>
  ///
  ///     <topic>~/elevator</topic>
  ///   </plugin>
  /// \endverbatim
  ///
  /// See worlds/elevator.world for a complete example.
  class GZ_PLUGIN_VISIBLE CustomElevatorPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: CustomElevatorPlugin();

    /// \brief Destructor.
    public: ~CustomElevatorPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited
    public: virtual void Reset();

    /// \brief Move to a particular floor.
    /// \param[in] _floor Number of the floor to move the elevator to.
    public: void MoveToFloor(const int _floor);

    /// \brief Update the plugin once every iteration of simulation.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Receives messages on the elevator's topic.
    /// \param[in] _msg The string message that contains a command.
    private: void OnElevator(ConstGzStringPtr &_msg);

    /// \brief Private data pointer
    private: std::unique_ptr<CustomElevatorPluginPrivate> dataPtr;
  };
}
#endif
