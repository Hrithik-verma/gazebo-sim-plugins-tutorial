#include "tutorial_gazebo_plugins/MoveModelTopicWay.hh"


using namespace gz;
using namespace sim;
using namespace systems;

MoveModelTopicWay::MoveModelTopicWay() = default;
MoveModelTopicWay::~MoveModelTopicWay() = default;

void MoveModelTopicWay::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // 1. Read the target model name from SDF
  if (!_sdf->HasElement("model_name"))
  {
    gzerr << "MoveModel plugin requires a <model_name> element." << std::endl;
    return;
  }
  this->modelName = _sdf->Get<std::string>("model_name");
  gzmsg << "Target Model Name: " << this->modelName << std::endl;

  if (!_sdf->HasElement("topic_name"))
  {
    gzerr << "MoveModel plugin requires a <topic_name> element." << std::endl;
    return;
  }

  this->topicName = _sdf->Get<std::string>("topic_name");
  gzmsg << "Cmd vel z Topic Name: " << this->topicName << std::endl;

  this->node.Subscribe(this->topicName, &MoveModelTopicWay::OnTransportMsg, this);

  if (_sdf->HasElement("z_velocity"))
  {
    this->zVelocity = _sdf->Get<double>("z_velocity");
  }
  gzmsg << "Initial Z Velocity: " << this->zVelocity << " m/s" << std::endl;


}

void MoveModelTopicWay::OnTransportMsg(const gz::msgs::Double & _msg){
  // Directly extract the double value from the message
  this->zVelocity = _msg.data();
  gzdbg << "Received new Z-velocity command: " << this->zVelocity << std::endl;
}

void MoveModelTopicWay::Update(const UpdateInfo &_info,
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
GZ_ADD_PLUGIN(gz::sim::systems::MoveModelTopicWay,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::MoveModelTopicWay,
                    "gz::sim::systems::MoveModelTopicWay")
