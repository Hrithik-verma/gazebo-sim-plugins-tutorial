

#include "PrintEntitySystemPlugin.hh"


using namespace gz;
using namespace sim;
using namespace systems;

PrintEntitySystemPlugin::PrintEntitySystemPlugin(){
  std::cout<<"PrintEntitySystemPlugin Plugin Started!!"<<std::endl;
}
PrintEntitySystemPlugin::~PrintEntitySystemPlugin(){
  std::cout<<"PrintEntitySystemPlugin Plugin stopped!!"<<std::endl;
}
void PrintEntitySystemPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->targetEntity = _entity;

  // means find the value of the ***Name Component** attached to **targetEntity** 
  auto Name = _ecm.Component<components::Name>(this->targetEntity);
  this->modelName = Name->Data();

  gzmsg << "Target Entity value: " << this->targetEntity << std::endl;
  gzmsg << "Target Entity Name: " << this->modelName << std::endl;


}

void PrintEntitySystemPlugin::PreUpdate(const UpdateInfo &_info,
                          EntityComponentManager &_ecm)
{
  // Only run if the simulation is not paused
  if (_info.paused)
    return;

  
}

// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::systems::PrintEntitySystemPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::PrintEntitySystemPlugin,
                    "gz::sim::systems::PrintEntitySystemPlugin")
