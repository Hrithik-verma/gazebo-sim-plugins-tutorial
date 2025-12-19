#include "tutorial_gazebo_plugins/LightControl.hh"

#include <gz/plugin/Register.hh>

#include <gz/msgs/light.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Name.hh>

#include <sdf/Light.hh>
#include <gz/sim/Conversions.hh>


using namespace gz;
using namespace gz::sim;


// Find all light entities
void LightControl::FindLightEntities(EntityComponentManager &_ecm)
{
  this->lightEntites.clear();

  _ecm.Each<components::Light, components::Name>(
      [&](const Entity &_entity,
          const components::Light *,
          const components::Name *_name) -> bool
      {
        this->lightEntites.push_back(_entity);
        // gzmsg << "Found light: " << _name->Data()
        //       << " (entity " << _entity << ")\n";
        return true;
      });
}

// ---------------------------------------------------------------------
void LightControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
   if (_info.paused)
    return;

  this->time += std::chrono::duration_cast<std::chrono::duration<double>>(
                    _info.dt)
                    .count();

  this->FindLightEntities(_ecm);
  if (this->lightEntites.empty())
    return;

  // Animated RGB in [0,1]
  const double r = 0.5 * (1.0 + std::sin(this->time * 0.5));
  const double g = 0.5 * (1.0 + std::sin(this->time * 0.5 + 2.0));
  const double b = 0.5 * (1.0 + std::sin(this->time * 0.5 + 4.0));
  gz::math::Color newColor(r, g, b, 1.0);

  for (const Entity e : this->lightEntites)
  {
    auto lightComp = _ecm.Component<components::Light>(e);
    if (!lightComp)
      continue;

    const sdf::v14::Light &sdfLight = lightComp->Data();

    gz::msgs::Light msg = gz::sim::convert<gz::msgs::Light>(sdfLight);


    gz::msgs::Set(msg.mutable_diffuse(),  newColor);
    gz::msgs::Set(msg.mutable_specular(), newColor);

    //method1
    // auto cmdComp = _ecm.Component<components::LightCmd>(e);
    // if (!cmdComp)
    //   _ecm.CreateComponent(e, components::LightCmd(msg));
    // else
    // {
      
    //   cmdComp->Data() = msg;

      
    // }

    //method2
    _ecm.SetComponentData<components::LightCmd>(e,msg);

    // in case of light we need to trigger update so that rendering system knows it updated
    _ecm.SetChanged(e,
                    components::LightCmd::typeId,
                    ComponentState::PeriodicChange);
                    /// PeriodicChange ->changing periodically,ok to drop few samples
  }
}

// Register the plugin
GZ_ADD_PLUGIN(
    LightControl,
    gz::sim::System,
    LightControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LightControl, "gz::sim::LightControl")
