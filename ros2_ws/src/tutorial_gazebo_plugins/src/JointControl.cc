#include "tutorial_gazebo_plugins/JointControl.hh"

#include <gz/plugin/Register.hh>


#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>

#include <algorithm>  // for std::clamp


using namespace gz;
using namespace gz::sim;

void JointControl::Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr)
{
  if (!_sdf->HasElement("joint_name"))
  {
    gzerr << "JointControl plugin requires <joint_name>" << std::endl;
    return;
  }

  this->jointName = _sdf->Get<std::string>("joint_name");

  if(!_ecm.EntityHasComponentType(_entity, components::Model::typeId)){
    gzerr << "JointControl plugin must be added on <model>...</model> scope only" << std::endl;
    return;
  }


  if (_sdf->HasElement("mode"))
  {
    const auto modeStr = _sdf->Get<std::string>("mode");
    if (modeStr == "velocity" || modeStr == "vel"){
      this->mode = Mode::Velocity;
      gzmsg << "Mode: Velocity" << std::endl;
    }
    
    else if(modeStr == "force" || modeStr == "f"){
      this->mode = Mode::Force;
      gzmsg << "Mode: Force" << std::endl;
    }
    else if (modeStr == "position" || modeStr == "pos"){
      this->mode = Mode::Position;
      gzmsg << "Mode: Position" << std::endl;
    }
    else
      gzerr << "JointControl: unknown <mode> '" << modeStr
            << "'. Using 'velocity'." << std::endl;
  }

  if (_sdf->HasElement("cmd_vel"))
    this->cmdVelocity = _sdf->Get<double>("cmd_vel");

  if (_sdf->HasElement("cmd_torque"))
    this->cmdTorque = _sdf->Get<double>("cmd_torque");

  if (_sdf->HasElement("pos_a"))
    this->posA = _sdf->Get<double>("pos_a");
  if (_sdf->HasElement("pos_b"))
    this->posB = _sdf->Get<double>("pos_b");

  if (_sdf->HasElement("p_gain"))
    this->pGain = _sdf->Get<double>("p_gain");

  if (_sdf->HasElement("d_gain"))
    this->dGain = _sdf->Get<double>("d_gain");

  if (_sdf->HasElement("max_torque"))
    this->maxTorque = _sdf->Get<double>("max_torque");
  
  if (_sdf->HasElement("pos_tolerence"))
    this->pTolerance = _sdf->Get<double>("pos_tolerence");

  if (_sdf->HasElement("wait_time"))
    this->waitTime = _sdf->Get<int>("wait_time");


  /* find such a joint entity which has all of the followings components:
        parent entity is model entity which plugin is attached to,
        has a name component of joint name 
        joint compenent is attach on it
        reference code: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/Model.cc#L132C1-L133C1
  */
  this->jointEntity = _ecm.EntityByComponents(
      components::ParentEntity(_entity),
      components::Name(jointName),
      components::Joint());

  this->targetPos = this->posA;   // start by going to posA
  this->lastSwitchTime = std::chrono::steady_clock::duration::zero(); // or set on first PreUpdate
}

// ---------------------------------------------------------------------
void JointControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
   if (_info.paused)
    return;

   if(!this->jointEntity)
     return;


  if (this->mode == Mode::Velocity)
  {
    if (!_ecm.EntityHasComponentType(this->jointEntity, components::JointVelocityCmd::typeId))
      _ecm.CreateComponent(this->jointEntity, components::JointVelocityCmd({0.0}));

    _ecm.SetComponentData<components::JointVelocityCmd>(this->jointEntity,
                                                          {this->cmdVelocity});
  }

  else if (this->mode == Mode::Force){
    
    //if component is not preset than create it
    if (!_ecm.EntityHasComponentType(this->jointEntity, components::JointForceCmd::typeId))
      _ecm.CreateComponent(this->jointEntity, components::JointForceCmd({0.0}));

    _ecm.SetComponentData<components::JointForceCmd>(this->jointEntity,
                                                          {this->cmdTorque});
  }

    
  else if (this->mode == Mode::Position){

    if (!this->hasInitTime)
    {
      this->lastSwitchTime = _info.simTime;
      this->hasInitTime = true;
    }

    
    // on give time interval will switch target position
    auto elapsed = _info.simTime - this->lastSwitchTime;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= this->waitTime)
    {
      this->targetPos = (this->targetPos == this->posA) ? this->posB : this->posA;
      this->lastSwitchTime = _info.simTime;
    }
    
    //read current position
    auto posComp = _ecm.Component<components::JointPosition>(this->jointEntity);
    if (!posComp)
    {
      _ecm.CreateComponent(this->jointEntity, components::JointPosition());
      gzdbg << "JointPosition component missing\n";
      return;
    }
    if (posComp->Data().empty())
    {
      gzdbg << "JointPosition data empty\n";
      return;
    }
    double currentPos = posComp->Data()[0];

    // Current velocity
    auto velComp = _ecm.Component<components::JointVelocity>(this->jointEntity);
    if (!velComp)
    {
      _ecm.CreateComponent(this->jointEntity, components::JointVelocity());
      gzdbg << "JointPosition component missing\n";
      return;
    }
    double currentVel = velComp->Data()[0];

    // PD control
    double error = this->targetPos - currentPos;
    double torque = this->pGain * error - this->dGain * currentVel;

    double maxT = this->maxTorque;        
    double clamp_torque = std::clamp(torque, -maxT, maxT);

    //once reached within the tolerance will stop applying torque
    if(error >= -pTolerance && error <= pTolerance){
      clamp_torque = 0.0;
    }

    // ensure force cmd exists (if you didn’t create in Configure)
    if (!_ecm.EntityHasComponentType(this->jointEntity, components::JointForceCmd::typeId))
      _ecm.CreateComponent(this->jointEntity, components::JointForceCmd({0.0}));

    _ecm.SetComponentData<components::JointForceCmd>(this->jointEntity, {clamp_torque});
    gzmsg << "pos target=" << this->targetPos << " cur=" << currentPos
          << " torque=" << clamp_torque << std::endl;

  }
  
}

// Register the plugin
GZ_ADD_PLUGIN(
    JointControl,
    gz::sim::System,
    JointControl::ISystemConfigure,
    JointControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(JointControl, "gz::sim::JointControl")
