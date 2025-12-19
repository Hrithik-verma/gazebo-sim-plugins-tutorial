#include "tutorial_gazebo_plugins/JointControl.hh"

#include <gz/plugin/Register.hh>

#include <gz/msgs/Utility.hh>

#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>

#include <sdf/Light.hh>
#include <gz/sim/Conversions.hh>


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
}

// ---------------------------------------------------------------------
void JointControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
   if (_info.paused)
    return;

   if(!this->jointEntity)
     return;

    //set a constant joint velocity
    std::vector<double> values; 
    values.push_back(1.0);
   _ecm.SetComponentData<components::JointVelocityCmd >(this->jointEntity, values);



  
}

// Register the plugin
GZ_ADD_PLUGIN(
    JointControl,
    gz::sim::System,
    JointControl::ISystemConfigure,
    JointControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(JointControl, "gz::sim::JointControl")
