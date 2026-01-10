#include <functional>

#include <ignition/common/Profiler.hh>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>

#include "CustomElevatorPluginPrivate.hh"
#include "CustomElevatorPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CustomElevatorPlugin)

/////////////////////////////////////////////////
CustomElevatorPlugin::CustomElevatorPlugin()
  : dataPtr(new CustomElevatorPluginPrivate)
{
  this->dataPtr->doorController = NULL;
  this->dataPtr->liftController = NULL;
  this->dataPtr->doorWaitTime = common::Time(5, 0);
}

/////////////////////////////////////////////////
CustomElevatorPlugin::~CustomElevatorPlugin()
{
  this->dataPtr->updateConnection.reset();

  delete this->dataPtr->doorController;
  this->dataPtr->doorController = NULL;

  delete this->dataPtr->liftController;
  this->dataPtr->liftController = NULL;
}

/////////////////////////////////////////////////
void CustomElevatorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "CustomElevatorPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "CustomElevatorPlugin sdf pointer is NULL");
  this->dataPtr->model = _model;
  this->dataPtr->sdf = _sdf;

  // Get the time to hold the door open.
  if (this->dataPtr->sdf->HasElement("door_wait_time"))
  {
    this->dataPtr->doorWaitTime.Set(
      this->dataPtr->sdf->Get<double>("door_wait_time"));
  }

  // Get the elevator topic.
  std::string elevatorTopic = "~/elevator";
  if (this->dataPtr->sdf->HasElement("topic"))
    elevatorTopic = this->dataPtr->sdf->Get<std::string>("topic");

  float floorHeight = 3.0;
  if (this->dataPtr->sdf->HasElement("floor_height"))
    floorHeight = this->dataPtr->sdf->Get<float>("floor_height");
  else
  {
    gzwarn << "No <floor_height> specified for elevator plugin. "
           << "Using a height of 3 meters. This value may cause "
           << "the elevator to move incorrectly.\n";
  }

  // Get the floor offset (added to each floor position calculation)
  float floorOffset = 0.0;
  if (this->dataPtr->sdf->HasElement("floor_offset"))
  {
    floorOffset = this->dataPtr->sdf->Get<float>("floor_offset");
    gzmsg << "Custom elevator plugin using floor_offset: " << floorOffset << "\n";
  }

  // Get the lift joint
  std::string liftJointName =
    this->dataPtr->sdf->Get<std::string>("lift_joint");

  this->dataPtr->liftJoint = this->dataPtr->model->GetJoint(liftJointName);
  if (!this->dataPtr->liftJoint)
  {
    gzerr << "Unable to find lift joint[" << liftJointName << "].\n";
    gzerr << "The elevator plugin is disabled.\n";
    return;
  }

  // Get the door joint
  std::string doorJointName =
    this->dataPtr->sdf->Get<std::string>("door_joint");

  this->dataPtr->doorJoint = this->dataPtr->model->GetJoint(doorJointName);
  if (!this->dataPtr->doorJoint)
  {
    gzerr << "Unable to find door joint[" << doorJointName << "].\n";
    gzerr << "The elevator plugin is disabled.\n";
    return;
  }

  // Create the door and lift controllers.
  this->dataPtr->doorController = new CustomElevatorPluginPrivate::DoorController(
      this->dataPtr->doorJoint);
  this->dataPtr->liftController = new CustomElevatorPluginPrivate::LiftController(
      this->dataPtr->liftJoint, floorHeight, floorOffset);

  // Connect to the update event.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CustomElevatorPlugin::Update, this, std::placeholders::_1));

  // Create the node for communication
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());

  // Subscribe to the elevator topic.
  this->dataPtr->elevatorSub = this->dataPtr->node->Subscribe(elevatorTopic,
      &CustomElevatorPlugin::OnElevator, this);
}

/////////////////////////////////////////////////
void CustomElevatorPlugin::OnElevator(ConstGzStringPtr &_msg)
{
  // Currently we only expect the message to contain a floor to move to.
  try
  {
    this->MoveToFloor(std::stoi(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Unable to process elevator message["
          << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void CustomElevatorPlugin::MoveToFloor(const int _floor)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  // Ignore messages when the elevator is currently busy.
  if (!this->dataPtr->states.empty())
    return;

  // Step 1: close the door.
  this->dataPtr->states.push_back(new CustomElevatorPluginPrivate::CloseState(
        this->dataPtr->doorController));

  // Step 2: Move to the correct floor.
  this->dataPtr->states.push_back(new CustomElevatorPluginPrivate::MoveState(
        _floor, this->dataPtr->liftController));

  // Step 3: Open the door
  this->dataPtr->states.push_back(new CustomElevatorPluginPrivate::OpenState(
        this->dataPtr->doorController));

  // Step 4: Wait
  this->dataPtr->states.push_back(new CustomElevatorPluginPrivate::WaitState(
        this->dataPtr->doorWaitTime));

  // Step 5: Close the door
  this->dataPtr->states.push_back(new CustomElevatorPluginPrivate::CloseState(
        this->dataPtr->doorController));
}

/////////////////////////////////////////////////
void CustomElevatorPlugin::Update(const common::UpdateInfo &_info)
{
  IGN_PROFILE("CustomElevatorPlugin::Update");
  IGN_PROFILE_BEGIN("Update");
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  // Process the states
  if (!this->dataPtr->states.empty())
  {
    // Update the front state, and remove it if the state is done
    if (this->dataPtr->states.front()->Update())
    {
      delete this->dataPtr->states.front();
      this->dataPtr->states.pop_front();
    }
  }

  // Update the controllers
  this->dataPtr->doorController->Update(_info);
  this->dataPtr->liftController->Update(_info);
  IGN_PROFILE_END();
}

////////////////////////////////////////////////
void CustomElevatorPlugin::Reset()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
  for (auto s: this->dataPtr->states)
    delete s;
  this->dataPtr->states.clear();
  this->dataPtr->doorController->Reset();
  this->dataPtr->liftController->Reset();
}

////////////////////////////////////////////////
// CustomElevatorPluginPrivate Class

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::~CustomElevatorPluginPrivate()
{
  delete this->doorController;
  this->doorController = NULL;

  delete this->liftController;
  this->liftController = NULL;

  for (auto s: this->states)
    delete s;
  this->states.clear();
}

////////////////////////////////////////////////
// CloseState Class

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::CloseState::CloseState(DoorController *_ctrl)
  : State(), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void CustomElevatorPluginPrivate::CloseState::Start()
{
  this->ctrl->SetTarget(CustomElevatorPluginPrivate::DoorController::CLOSE);
  this->started = true;
}

/////////////////////////////////////////////////
bool CustomElevatorPluginPrivate::CloseState::Update()
{
  IGN_PROFILE("CustomElevatorPlugin::CloseState");
  IGN_PROFILE_BEGIN("Update");

  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetTarget() ==
      CustomElevatorPluginPrivate::DoorController::CLOSE &&
      this->ctrl->GetState() ==
      CustomElevatorPluginPrivate::DoorController::STATIONARY;
  }
  IGN_PROFILE_END();
}

////////////////////////////////////////////////
// OpenState Class

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::OpenState::OpenState(DoorController *_ctrl)
  : State(), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void CustomElevatorPluginPrivate::OpenState::Start()
{
  this->ctrl->SetTarget(CustomElevatorPluginPrivate::DoorController::OPEN);
  this->started = true;
}

/////////////////////////////////////////////////
bool CustomElevatorPluginPrivate::OpenState::Update()
{
  IGN_PROFILE("CustomElevatorPlugin::OpenState");
  IGN_PROFILE_BEGIN("Update");

  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetTarget() ==
      CustomElevatorPluginPrivate::DoorController::OPEN &&
      this->ctrl->GetState() ==
      CustomElevatorPluginPrivate::DoorController::STATIONARY;
  }
  IGN_PROFILE_END();
}

////////////////////////////////////////////////
// MoveState Class

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::MoveState::MoveState(int _floor, LiftController *_ctrl)
  : State(), floor(_floor), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void CustomElevatorPluginPrivate::MoveState::Start()
{
  this->ctrl->SetFloor(this->floor);
  this->started = true;
}

/////////////////////////////////////////////////
bool CustomElevatorPluginPrivate::MoveState::Update()
{
  IGN_PROFILE("CustomElevatorPlugin::MoveState");
  IGN_PROFILE_BEGIN("Update");

  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetState() ==
      CustomElevatorPluginPrivate::LiftController::STATIONARY;
  }
  IGN_PROFILE_END();
}

////////////////////////////////////////////////
// WaitState Class

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::WaitState::WaitState(const common::Time &_waitTime)
  : State(), waitTimer(_waitTime, true)
{
}

/////////////////////////////////////////////////
void CustomElevatorPluginPrivate::WaitState::Start()
{
  this->waitTimer.Reset();
  this->waitTimer.Start();
  this->started = true;
}

/////////////////////////////////////////////////
bool CustomElevatorPluginPrivate::WaitState::Update()
{
  IGN_PROFILE("CustomElevatorPlugin::WaitState");
  IGN_PROFILE_BEGIN("Update");

  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    if (this->waitTimer.GetElapsed() == common::Time::Zero)
      return true;
    else
      return false;
  }
  IGN_PROFILE_END();
}

////////////////////////////////////////////////
// DoorController Class

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::DoorController::DoorController(
    physics::JointPtr _doorJoint)
  : doorJoint(_doorJoint), state(STATIONARY), target(CLOSE)
{
  this->doorPID.Init(2, 0, 1.0);
}

/////////////////////////////////////////////////
void CustomElevatorPluginPrivate::DoorController::SetTarget(
    CustomElevatorPluginPrivate::DoorController::Target _target)
{
  this->target = _target;
}

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::DoorController::Target
CustomElevatorPluginPrivate::DoorController::GetTarget() const
{
  return this->target;
}

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::DoorController::State
CustomElevatorPluginPrivate::DoorController::GetState() const
{
  return this->state;
}

/////////////////////////////////////////////////
void CustomElevatorPluginPrivate::DoorController::Reset()
{
  this->prevSimTime = common::Time::Zero;
}

/////////////////////////////////////////////////
bool CustomElevatorPluginPrivate::DoorController::Update(
    const common::UpdateInfo &_info)
{
  IGN_PROFILE("CustomElevatorPlugin::DoorController");
  IGN_PROFILE_BEGIN("Update");

  // Bootstrap the time.
  if (this->prevSimTime == common::Time::Zero)
  {
    this->prevSimTime = _info.simTime;
    return false;
  }

  double errorTarget = this->target == OPEN ? 1.0 : 0.0;

  double doorError = this->doorJoint->Position() -
    errorTarget;

  double doorForce = this->doorPID.Update(doorError,
      _info.simTime - this->prevSimTime);

  this->doorJoint->SetForce(0, doorForce);

  if (std::abs(doorError) < 0.05)
  {
    this->state = STATIONARY;
    return true;
  }
  else
  {
    this->state = MOVING;
    return false;
  }
  IGN_PROFILE_END();
}

////////////////////////////////////////////////
// LiftController Class

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::LiftController::LiftController(
    physics::JointPtr _liftJoint, float _floorHeight, float _floorOffset)
  : state(STATIONARY), floor(0), floorHeight(_floorHeight),
    floorOffset(_floorOffset), liftJoint(_liftJoint)
{
  this->liftPID.Init(100000, 0, 100000.0);
}

/////////////////////////////////////////////////
void CustomElevatorPluginPrivate::LiftController::Reset()
{
  this->prevSimTime = common::Time::Zero;
}

/////////////////////////////////////////////////
bool CustomElevatorPluginPrivate::LiftController::Update(
    const common::UpdateInfo &_info)
{
  IGN_PROFILE("CustomElevatorPlugin::LiftController");
  IGN_PROFILE_BEGIN("Update");

  // Bootstrap the time.
  if (this->prevSimTime == common::Time::Zero)
  {
    this->prevSimTime = _info.simTime;
    return false;
  }

  double error = this->liftJoint->Position() -
    (this->floor * this->floorHeight + this->floorOffset);

  double force = this->liftPID.Update(error, _info.simTime - this->prevSimTime);
  this->prevSimTime = _info.simTime;

  this->liftJoint->SetForce(0, force);

  if (std::abs(error) < 0.15)
  {
    this->state = CustomElevatorPluginPrivate::LiftController::STATIONARY;
    return true;
  }
  else
  {
    this->state = CustomElevatorPluginPrivate::LiftController::MOVING;
    return false;
  }
  IGN_PROFILE_END();
}

/////////////////////////////////////////////////
void CustomElevatorPluginPrivate::LiftController::SetFloor(int _floor)
{
  this->floor = _floor;
}

/////////////////////////////////////////////////
int CustomElevatorPluginPrivate::LiftController::GetFloor() const
{
  return this->floor;
}

/////////////////////////////////////////////////
CustomElevatorPluginPrivate::LiftController::State
CustomElevatorPluginPrivate::LiftController::GetState() const
{
  return this->state;
}
