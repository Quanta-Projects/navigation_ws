#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <iostream>

namespace gazebo
{
  class JointMoverPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;
      this->joint = this->model->GetJoint("moving_joint");

      if (!this->joint)
      {
        gzerr << "Joint not found!" << std::endl;
        return;
      }

      // Update the joint position in each simulation iteration
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointMoverPlugin::OnUpdate, this));

      std::cout << "JointMoverPlugin loaded successfully!" << std::endl;
    }

    void OnUpdate()
    {
      if (!this->joint)
        return;

      // Simple oscillating motion
      double time = this->model->GetWorld()->SimTime().Double();
      double angle = sin(time); // Oscillates between -1 and 1 radian
      this->joint->SetPosition(0, angle);
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(JointMoverPlugin)
}
