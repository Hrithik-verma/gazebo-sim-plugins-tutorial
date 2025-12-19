#include "tutorial_gazebo_plugins/LightControlTopicWay.hh"

#include <chrono>
#include <cmath>
#include <iostream>

#include <gz/common/Console.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/light.pb.h>

#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/Conversions.hh>
#include <gz/math/Color.hh>
#include <gz/plugin/Register.hh> //for GZ_ADD_PLUGIN_ALIAS()


using namespace gz;
using namespace gz::sim;

//////////////////////////////////////////////////
void LightControlTopicWay::Configure(const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  if (!_sdf || !_sdf->HasElement("on_off_service_name"))
  {
    gzerr << "LightControlTopicWay plugin requires <on_off_service_name>."
          << std::endl;
    return;
  }

  this->serviceName = _sdf->Get<std::string>("on_off_service_name");
 

  // Initial state
  this->isLightOn = true;
  this->pendingApply = true;  // push initial state to the world once

  
  // Advertise service: Empty -> Empty, callback returns bool
  bool ok = this->node.Advertise(
      this->serviceName,
      &LightControlTopicWay::srvLightToggle,
      this);

  if (!ok)
  {
    std::cerr << "Error advertising service [" << this->serviceName << "]"
              << std::endl;
  }
  
   gzmsg << "[LightControlTopicWay] Service Name: "
        << this->serviceName << std::endl;


}

//////////////////////////////////////////////////
bool LightControlTopicWay::srvLightToggle(
    const gz::msgs::Empty &/*_req*/, gz::msgs::Empty &/*_rep*/)
{
  // Flip state and mark that we must push a single command if turning OFF
  this->isLightOn = !this->isLightOn;
  this->pendingApply = true;

  gzmsg << "[LightControlTopicWay] Toggled. isLightOn = " << std::boolalpha << this->isLightOn << std::endl;

  std::cerr << "[LightControlTopicWay] Toggled. isLightOn = "
        << std::boolalpha << this->isLightOn << std::endl;
  return true;
}

//////////////////////////////////////////////////
void LightControlTopicWay::FindLightEntities(EntityComponentManager &_ecm)
{
  this->lightEntities.clear();
  _ecm.Each<components::Light>(
      [&](const Entity &_e, const components::Light *) -> bool
      {
        this->lightEntities.push_back(_e);
        return true;
      });
}

//////////////////////////////////////////////////
void LightControlTopicWay::PreUpdate(const UpdateInfo &_info,
                                     EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // Only advance animation time while ON
  if (this->isLightOn)
  {
    this->time += std::chrono::duration_cast<std::chrono::duration<double>>(
                         _info.dt).count();
  }

  this->FindLightEntities(_ecm);
  if (this->lightEntities.empty())
    return;

  // Color for ON animation (computed only if ON)
  gz::math::Color animColor(1, 1, 1, 1);
  if (this->isLightOn)
  {
    const double r = 0.5 * (1.0 + std::sin(this->time * 0.5));
    const double g = 0.5 * (1.0 + std::sin(this->time * 0.5 + 2.0));
    const double b = 0.5 * (1.0 + std::sin(this->time * 0.5 + 4.0));
    animColor = gz::math::Color(r, g, b, 1.0);
  }

  for (const Entity e : this->lightEntities)
  {
    // When OFF and no state change to apply, do nothing at all
    if (!this->isLightOn && !this->pendingApply)
      continue;

    auto lightComp = _ecm.Component<components::Light>(e);
    if (!lightComp)
      continue;

    // Start from current SDF, then apply our command
    const sdf::v14::Light &sdfLight = lightComp->Data();
    gz::msgs::Light msg = gz::sim::convert<gz::msgs::Light>(sdfLight);

    if (this->isLightOn)
    {
      // While ON: animate color every tick and ensure light is enabled
      gz::msgs::Set(msg.mutable_diffuse(),  animColor);
      gz::msgs::Set(msg.mutable_specular(), animColor);
      msg.set_is_light_off(false);
      // Intensity left as configured by SDF; can be animated if desired
    }
    else
    {
      // Transitioning to OFF: send ONCE, then stop publishing
      msg.set_is_light_off(true);
      msg.set_intensity(0.0);   // defensive: ensure no residual emission
    }

    _ecm.SetComponentData<components::LightCmd>(e, msg);
    _ecm.SetChanged(e, components::LightCmd::typeId,
                    ComponentState::PeriodicChange);
  }

  if (!this->isLightOn && this->pendingApply)
    this->pendingApply = false;

}

// --- Plugin registration ---
GZ_ADD_PLUGIN(
  gz::sim::LightControlTopicWay,
  gz::sim::System,
  gz::sim::LightControlTopicWay::ISystemConfigure,
  gz::sim::LightControlTopicWay::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::LightControlTopicWay,
                    "gz::sim::LightControlTopicWay")
