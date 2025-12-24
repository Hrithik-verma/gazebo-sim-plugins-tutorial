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


  //check for such components which has Light, Name components
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

// // ---------------------------------------------------------------------
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

  //Un comment to see the flicker effect
  // Flicker (no RNG): irregular brightness 0.15..1.0
  constexpr double kTwoPi = 6.283185307179586;
  const double t = this->time;

  double flicker = 0.55
    + 0.25 * std::sin(kTwoPi * 17.0 * t)
    + 0.15 * std::sin(kTwoPi * 23.0 * t + 1.7)
    + 0.10 * std::sin(kTwoPi * 31.0 * t + 0.3);


  flicker = std::clamp(flicker, 0.15, 1.0);

  // // Animated RGB in [0,1]
  const double r = 0.5 * (1.0 + std::sin(this->time * 0.5)) * flicker;
  const double g = 0.5 * (1.0 + std::sin(this->time * 0.5 + 2.0)) * flicker;
  const double b = 0.5 * (1.0 + std::sin(this->time * 0.5 + 4.0)) * flicker;
  gz::math::Color newColor(r, g, b, 1.0);

  // Animated RGB in [0,1]
  // const double r = 0.5 * (1.0 + std::sin(this->time * 0.5));
  // const double g = 0.5 * (1.0 + std::sin(this->time * 0.5 + 2.0));
  // const double b = 0.5 * (1.0 + std::sin(this->time * 0.5 + 4.0));
  // gz::math::Color newColor(r, g, b, 1.0);


 
  for (const Entity e : this->lightEntites)
  {

    //read data of Light
    auto lightComp = _ecm.Component<components::Light>(e);
    if (!lightComp)
      continue;

    const sdf::v14::Light &sdfLight = lightComp->Data();

    //convert sdf light msg to gz light msg
    gz::msgs::Light msg = gz::sim::convert<gz::msgs::Light>(sdfLight);

    //using Set() to set the fields of light msg
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


    ///method 3 doesnot work as expected why?
    // auto lightObj = Light(e);
    // lightObj.SetSpecularColor(_ecm, newColor);
    // lightObj.SetDiffuseColor(_ecm, newColor); 
      

    /*
      https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/Light.cc check actual code 
      the problem is here it created a entire new msg means new light 
      because of which we can change our lights certain color property rather end up making new light
    
    */

    // in case of light we need to trigger update so that rendering system knows it updated
    _ecm.SetChanged(e,
                    components::LightCmd::typeId,
                    ComponentState::PeriodicChange);
                    // PeriodicChange ->changing periodically,ok to drop few samples
  }
}




// Register the plugin
GZ_ADD_PLUGIN(
    LightControl,
    gz::sim::System,
    LightControl::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LightControl, "gz::sim::LightControl")
