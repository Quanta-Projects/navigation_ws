#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>
#include <thread>
#include <chrono>
#include <ctime>
#include <cmath>

namespace gazebo
{
  class MultiFloorDoorControl : public ModelPlugin
  {
  public:
    // Constructor
    MultiFloorDoorControl() : door_opened(false), targetJoint(nullptr), floorRequest(0), elevatorStep(0.004), initialReading(true) {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;

      // Get joints
      this->joint1 = this->model->GetJoint("joint1");
      this->joint2 = this->model->GetJoint("joint2");
      this->joint3 = this->model->GetJoint("joint3");
      this->joint4 = this->model->GetJoint("joint4");
      this->elevator_joint = this->model->GetJoint("elevator_joint");


      if (!this->joint1 || !this->joint2 || !this->joint3 || !this->joint4 || !this->elevator_joint)
      {
        gzerr << "One or more joints not found!" << std::endl;
        return;
      }

      // Initialize Gazebo transport
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      // Subscribe to the ~/elevator topic
      this->sub = this->node->Subscribe("~/elevator", &MultiFloorDoorControl::OnMsg, this);

      std::cout << "MultiFloorDoorControl loaded successfully!" << std::endl;

      // Start a timer to update the control loop
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MultiFloorDoorControl::OnUpdate, this));
    }

    void OnMsg(ConstIntPtr &msg)
    {
      int command = msg->data();
      this->targetJoint = nullptr;  // Reset target joint
      double velocity = 0.3; // The velocity to set for the active joint

      switch (command)
      {
        case 0:
          this->targetJoint = this->joint1;
          this->floorRequest = 0;
          //this->elevator_joint->SetPosition(0, 0.0); 
          break;

        case 1:
          this->targetJoint = this->joint2;
          this->floorRequest = 1;
          //this->elevator_joint->SetPosition(0, 3.0);
          break;

        case 2:
          this->targetJoint = this->joint3;
          this->floorRequest = 2;
          //this->elevator_joint->SetPosition(0, 6.0);
          break;

        case 3:
          this->targetJoint = this->joint4;
          this->floorRequest = 3;
          //this->elevator_joint->SetPosition(0, 9.0);
          break;

        case 4:
          this->floorRequest = 4;
          this->targetJoint = nullptr; // Stop all joints
          break;

        case 5:
          // Set reverse velocity for all joints
          this->SetJointVelocity(this->joint1, -velocity);
          this->SetJointVelocity(this->joint2, -velocity);
          this->SetJointVelocity(this->joint3, -velocity);
          this->SetJointVelocity(this->joint4, -velocity);
          break;

        default:
          std::cerr << "Unknown command: " << command << std::endl;
          break;
      }
    }

    void OnUpdate()
    {
      if (this->targetJoint)
      {
        double elevatorPosition = this->elevator_joint->Position(2);
        //std::cout<< "Elevator position: "<<elevatorPosition <<std::endl;
        double elevationGap = abs(elevatorPosition - this->floorRequest*3);
        //std::cout<< "Elevator Gap: "<<elevationGap <<std::endl;
        double elevationFactor = 1;
        if(elevationGap < 1.0)
        {
          elevationFactor = elevationGap;
        }

        if (elevatorPosition > this->floorRequest*3 + 0.05)
        {
            this->elevator_joint->SetPosition(0, elevatorPosition - (elevatorStep * elevationFactor)/10);  // Going down
        }
        else if(elevatorPosition < this->floorRequest*3 - 0.05)
        {
            this->elevator_joint->SetPosition(0, elevatorPosition + (elevatorStep * elevationFactor));     // Going up
        }
        else
        {   
            if(this->floorRequest != 0){
            this->elevator_joint->SetPosition(0, this->floorRequest*3 - 0.038); 
            }
            //std::cout<< "Elevator reached targetted floor! "<<std::endl;

            double desiredPosition = this->door_opened ? 0.0 : 1.0; // Determine desired position based on current state 0.8
            double currentPosition = this->targetJoint->Position(0);
            // Move toward the desired position
            if ((this->door_opened && currentPosition > desiredPosition) || 
                (!this->door_opened && currentPosition < desiredPosition))
            {
              double velocity = this->door_opened ? -0.3 : 0.3; // Set velocity based on direction
              this->SetJointVelocity(this->targetJoint, velocity);
            }
            else
            {
              // Stop the joint once the desired position is reached
              this->SetJointVelocity(this->targetJoint, 0.0);
              //this->door_opened = !this->door_opened; // Toggle state
              if(this->initialReading)
              {
                this->initTimestamp = std::chrono::system_clock::now();
                this->initialReading = !this->initialReading;
              }
              else
              {
                this->endTimestamp = std::chrono::system_clock::now();
                this->duration     = this->endTimestamp - this->initTimestamp;
                if(this->duration.count() > 10.0) // 10.0
                {
                  //std::this_thread::sleep_for(std::chrono::seconds(3));
                  this->door_opened = !this->door_opened; // Toggle state
                  //std::cout << (this->door_opened ? "Door Opened!" : "Door Closed!") << std::endl;
                  if(!this->door_opened){
                    this->targetJoint = nullptr; // Stop controlling the joint after reaching the target
                    this->initialReading = !this->initialReading;
                  }
                }
                
              }
              
            }
        }
      }
      else
      {
        // Stop all joints if no target joint is set
        if(this->floorRequest != 0){
            this->elevator_joint->SetPosition(0, this->floorRequest*3 - 0.038); 
            }
        this->SetJointVelocity(this->joint1, 0.0);
        this->SetJointVelocity(this->joint2, 0.0);
        this->SetJointVelocity(this->joint3, 0.0);
        this->SetJointVelocity(this->joint4, 0.0);
      }
      
    }

  private:
    void SetJointVelocity(physics::JointPtr joint, double velocity ,int axis = 0)
    {
      if (!joint) return;
      joint->SetVelocity(axis, velocity); // Set the velocity of the joint
    }

    physics::ModelPtr model;
    physics::JointPtr joint1;
    physics::JointPtr joint2;
    physics::JointPtr joint3;
    physics::JointPtr joint4;
    physics::JointPtr elevator_joint;
    physics::JointPtr targetJoint; // Pointer to the current target joint
    transport::NodePtr node;
    transport::SubscriberPtr sub;
    event::ConnectionPtr updateConnection; // Connection to the update event
    bool door_opened; // State variable to track door status
    int floorRequest;
    double elevatorStep;
    bool initialReading;
    std::chrono::time_point<std::chrono::system_clock> initTimestamp, endTimestamp;
    std::chrono::duration<double> duration;

  };

  GZ_REGISTER_MODEL_PLUGIN(MultiFloorDoorControl)
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
