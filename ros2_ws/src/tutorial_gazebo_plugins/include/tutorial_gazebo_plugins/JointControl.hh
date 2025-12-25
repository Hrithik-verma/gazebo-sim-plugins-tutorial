#ifndef JOINT_CONTROL_PLUGIN_HH_
#define JOINT_CONTROL_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <vector>
#include <string>


enum class Mode
{
  Force,
  Velocity,
  Position
};

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

  Mode mode{Mode::Velocity};
  // Velocity mode
  double cmdVelocity{0.0}; // for Velcoity Mode
  double cmdTorque{0.0}; // for Force Mode
  // Position mode: oscillate between 2 targets // Position Mode
  double posA{0.0};
  double posB{0.0};
  int waitTime{1}; //time to wait from pos A->B
  double targetPos{0.0}; // current target pose
  bool hasInitTime{false}; // for init time
  double pGain{50.0};  // P 
  double dGain{1.0};   // D
  double maxTorque{10.0}; // max torque
  double pTolerance{0.01}; // position tolerance 
  std::chrono::steady_clock::duration lastSwitchTime{0}; //last time when switching happended

};
}  // namespace sim
}  // namespace gz

#endif
