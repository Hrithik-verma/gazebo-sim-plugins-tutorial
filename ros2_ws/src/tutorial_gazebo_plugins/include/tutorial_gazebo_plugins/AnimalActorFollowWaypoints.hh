#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components.hh>
#include <gz/sim/Actor.hh>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh> //for GZ_ADD_PLUGIN_ALIAS()

#include <chrono>
#include <cmath>
#include <vector>

namespace gz::sim
{

class AnimalActorFollowWaypoints :
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  void Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &) override;

  void PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm) override;

private:
  Entity actorEntity{kNullEntity};

  std::vector<math::Pose3d> waypoints;
  size_t currentWaypoint{0};
  std::chrono::steady_clock::time_point lastWallPrint{};
  double yawOffset;
  double zOffset{0.0};


  double speed{1.0}; // meters / second
};

}
