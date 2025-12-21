#include "MoveModel.hh"


using namespace gz;
using namespace sim;
using namespace systems;

MoveModel::MoveModel() = default;
MoveModel::~MoveModel() = default;

void MoveModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  if (!_sdf->HasElement("model_name"))
  {
    gzerr << "ApplyZVelocityPlugin requires a <model_name> element."
          << std::endl;
    return;
  }
  this->modelName = _sdf->Get<std::string>("model_name");
  gzmsg << "Target Model Name: " << this->modelName << std::endl;

  // Read the desired Z linear velocity from SDF
  if (!_sdf->HasElement("z_velocity"))
  {
    gzerr << "ApplyZVelocityPlugin requires a <z_velocity> element."
          << std::endl;
    return;
  }
  this->zVelocity = _sdf->Get<double>("z_velocity");
  gzmsg << "Desired Z Velocity: " << this->zVelocity << " m/s" << std::endl;
}

void MoveModel::PreUpdate(const UpdateInfo &_info,
                          EntityComponentManager &_ecm)
{
  // Only run if the simulation is not paused
  if (_info.paused)
    return;

  // 1. Find the target model entity by name (if not found yet)
  if (this->targetEntity == kNullEntity)
  {
    auto entityOpt = _ecm.EntityByName(this->modelName);
    if (!entityOpt.has_value())
    {
      gzdbg << "Model [" << this->modelName
            << "] not found yet. Skipping velocity application." << std::endl;
      return;
    }

    this->targetEntity = entityOpt.value();
    gzmsg << "Found target model entity: " << this->targetEntity << std::endl;
  }

  // 2. Get / create LinearVelocity component
//   auto velComp =
//       _ecm.Component<components::LinearVelocityCmd>(this->targetEntity);

//   if (!velComp)
//   {
//     velComp = _ecm.CreateComponent(this->targetEntity,
//                                    components::LinearVelocityCmd());
//     gzmsg << "Added LinearVelocity component to model: "
//           << this->modelName << std::endl;
//   }

//   if (!velComp)
//   {
//     gzerr << "Failed to create/get LinearVelocity component for model ["
//           << this->modelName << "]." << std::endl;
//     return;
//   }

//   // 3. Set the Z-axis linear velocity: (0, 0, zVelocity)
  const gz::math::Vector3d vel(0.0, 0.0, this->zVelocity);
//   velComp->Data() = vel;

    _ecm.SetComponentData<components::LinearVelocityCmd>(this->targetEntity,{vel});
}

// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::systems::MoveModel,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::MoveModel,
                    "gz::sim::systems::MoveModel")
