#include "tutorial_gazebo_plugins/AnimalActorFollowWaypoints.hh"

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
void AnimalActorFollowWaypoints::Configure(
  const Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm,
  EventManager &)
{
  // Find actor
  // _ecm.Each<components::Actor>(
  //   [&](const Entity &_ent, const components::Actor *) -> bool
  //   {
  //     this->actorEntity = _ent;
  //     return false;
  //   });

  // or

  this->actorEntity = _entity;

  if (this->actorEntity == kNullEntity)
  {
    gzerr << "[AnimalActorFollowWaypoints] No Actor found\n";
    return;
  }

   if (!_ecm.Component<components::Actor>(this->actorEntity))
  {
    gzerr << "[AnimalActorFollowWaypoints] Plugin attached to a non-actor entity\n";
    return;
  }

  // Read waypoints
  for (auto elem = _sdf->FindElement("waypoint");
       elem; elem = elem->GetNextElement("waypoint"))
  {
    this->waypoints.push_back(elem->Get<math::Pose3d>());
  }

  if (_sdf->HasElement("speed"))
    this->speed = _sdf->Get<double>("speed");

  if (this->waypoints.empty())
    gzerr << "[AnimalActorFollowWaypoints] No waypoints provided\n";
  
  if (_sdf->HasElement("yaw_offset"))
    this->yawOffset = _sdf->Get<double>("yaw_offset");

  if (_sdf->HasElement("z_offset"))
    this->zOffset = _sdf->Get<double>("z_offset");

  gzdbg << "[AnimalActorFollowWaypoints] Configure: actorEntity=" << this->actorEntity
      << " waypoints=" << this->waypoints.size()
      << " speed=" << this->speed << "\n";
}

void AnimalActorFollowWaypoints::PreUpdate(
  const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (this->waypoints.empty())
    return;

  // Throttle debug prints (~1 Hz)
  const auto now = std::chrono::steady_clock::now();
  const bool doPrint =
    (this->lastWallPrint.time_since_epoch().count() == 0) ||
    (now - this->lastWallPrint > std::chrono::seconds(1));
  if (doPrint)
    this->lastWallPrint = now;


  // auto actor = _ecm.Component<components::Actor>(this->actorEntity);
  // if(!actor)
  // or below method

  gz::sim::Actor actor(this->actorEntity);
  if (!actor.Valid(_ecm))
  {
    if (doPrint)
      gzerr << "[AnimalActorFollowWaypoints] PreUpdate: entity " << this->actorEntity
            << " not a valid Actor\n";
    return;
  }

  // auto originOpt = _ecm.Component<components::Pose>(this->actorEntity);
  // or below method

  // Origin pose (trajectory reference)
  auto originOpt = actor.Pose(_ecm);
  if (!originOpt)
  {
    if (doPrint)
      gzdbg << "[AnimalActorFollowWaypoints] Waiting for actor origin Pose component...\n";
    return;
  }
  const math::Pose3d origin = *originOpt;

  // TrajectoryPose
  math::Pose3d traj = math::Pose3d::Zero;
  if (auto trajComp = _ecm.Component<components::TrajectoryPose>(this->actorEntity))
    traj = trajComp->Data();

  // Derive current actor world pose from origin and trajectory
  math::Pose3d world = origin * traj;

  //current waypoint target
  const math::Pose3d targetWorld = this->waypoints[this->currentWaypoint];

  // Direction in world frame
  math::Vector3d dir = targetWorld.Pos() - world.Pos();
  dir.Z(0);                       // ignore Z for motion 
  const double dist = dir.Length();

  if (doPrint)
  {
    gzdbg << "[AnimalActorFollowWaypoints] Tick entity=" << this->actorEntity
          << " wp=" << this->currentWaypoint << "/" << this->waypoints.size()
          << " dt=" << std::chrono::duration<double>(_info.dt).count()
          << " origin.pos=" << origin.Pos()
          << " world.pos=" << world.Pos()
          << " traj.pos=" << traj.Pos()
          << " target.pos=" << targetWorld.Pos()
          << " dist=" << dist
          << "\n";
  }

  // Advance waypoint
  if (dist < 0.05)
  {
    this->currentWaypoint = (this->currentWaypoint + 1) % this->waypoints.size();
    if (doPrint)
      gzdbg << "[AnimalActorFollowWaypoints] Reached waypoint, advancing to wp="
            << this->currentWaypoint << "\n";
    return;
  }

  if (dist > 1e-9)
    dir.Normalize();

  //this->speed * dt means  Distance covered in one physics frame
  const double dt = std::chrono::duration<double>(_info.dt).count();
  const double step = this->speed * dt;

  // Step in world
  world.Pos() += dir * step;

  // Keep a constant height (or waypoint height) + offset
  world.Pos().Z() = targetWorld.Pos().Z() + this->zOffset;

  // Face direction
  const double yaw = atan2(dir.Y(), dir.X());
  world.Rot() = math::Quaterniond(0, 0, yaw + this->yawOffset);

  // Convert desired world pose -> trajectory (relative to origin)
  // world = origin * traj  =>  traj = origin^-1 * world
  const math::Pose3d trajCmd = origin.Inverse() * world;

  // auto pose = _ecm.Component<components::TrajectoryPose>(this->actorEntity);
  /*
    if (!pose)
    {
      _ecm.CreateComponent(
          this->dataPtr->id,
          components::TrajectoryPose(trajCmd));
    }
    else
    {
      pose->Data() = trajCmd;
    }
  */
  // or below method

  // Set manual trajectory pose
  actor.SetTrajectoryPose(_ecm, trajCmd);

  // Force notify that this component changed (this is the key)
  _ecm.SetChanged(this->actorEntity,
                  gz::sim::components::TrajectoryPose::typeId,
                  gz::sim::ComponentState::OneTimeChange);


  if (doPrint)
  {
    gzdbg << "[AnimalActorFollowWaypoints] SetTrajectoryPose traj(before)=" << traj.Pos()
          << " trajCmd(after)=" << trajCmd.Pos()
          << " world(after)=" << world.Pos()
          << " yaw=" << yaw
          << "\n";
  }
}
// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::AnimalActorFollowWaypoints,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::AnimalActorFollowWaypoints,
                    "gz::sim::systems::AnimalActorFollowWaypoints")