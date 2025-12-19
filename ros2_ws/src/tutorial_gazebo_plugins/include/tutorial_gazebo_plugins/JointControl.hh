#ifndef JOINT_CONTROL_PLUGIN_HH_
#define JOINT_CONTROL_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <vector>
#include <string>

namespace gz
{
namespace sim
{
class JointControl : public System, public ISystemConfigure,public ISystemPreUpdate
{
public:
  // Constructor
  JointControl() = default;

  void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  // Function to find the light entity by name
  std::string jointName; //joint name
  Entity jointEntity; //joint entity

};
}  // namespace sim
}  // namespace gz

#endif
