#include "MoveModelTopicWayPimp.hh"

#include <gz/transport/Node.hh> //for gz transport node
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h> // Include the Double message type
#include "gz/sim/Model.hh"  // for Model component
#include "gz/sim/components/LinearVelocity.hh" // for linear velocity 
#include "gz/sim/components/LinearVelocityCmd.hh" // for LinearVelocityCmd component
#include "gz/sim/components/Name.hh"  // for Name component

#include <gz/plugin/Register.hh> //for GZ_ADD_PLUGIN_ALIAS()
#include <gz/msgs/Utility.hh>    //for msg

using namespace gz;
using namespace sim;
using namespace systems;



class gz::sim::systems::MoveModelTopicWayPimpPrivate{

  public: //class data memeber, memeber function all need to be public for other class to use it
    std::string modelName;
    std::string topicName;
    double zVelocity{0.0};
    Entity targetEntity{kNullEntity};
    gz::transport::Node node; // GZ Transport node

    /* transport msg callback */
    void OnTransportMsg(const gz::msgs::Double & _msg);

};


MoveModelTopicWayPimp::MoveModelTopicWayPimp(): dataPtr(std::make_unique<MoveModelTopicWayPimpPrivate>())
{
  std::cout<<"MoveModelTopicWayPimp Plugin Started!!"<<std::endl;

}
MoveModelTopicWayPimp::~MoveModelTopicWayPimp(){
  std::cout<<"MoveModelTopicWayPimp Plugin stopped!!"<<std::endl;

}

void MoveModelTopicWayPimp::Configure(const Entity &_entity,
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
  this->dataPtr->modelName = _sdf->Get<std::string>("model_name");
  gzmsg << "Target Model Name: " << this->dataPtr->modelName << std::endl;

  if (!_sdf->HasElement("topic_name"))
  {
    gzerr << "MoveModel plugin requires a <topic_name> element." << std::endl;
    return;
  }

 this->dataPtr->topicName = _sdf->Get<std::string>("topic_name");
  gzmsg << "Cmd vel z Topic Name: " << this->dataPtr->topicName << std::endl;

  this->dataPtr->node.Subscribe(this->dataPtr->topicName, &MoveModelTopicWayPimpPrivate::OnTransportMsg, this->dataPtr.get()); //dataPtr.get() returns this pointer

  if (_sdf->HasElement("z_velocity"))
  {
    this->dataPtr->zVelocity = _sdf->Get<double>("z_velocity");
  }
  gzmsg << "Initial Z Velocity: " << this->dataPtr->zVelocity << " m/s" << std::endl;


}

void MoveModelTopicWayPimpPrivate::OnTransportMsg(const gz::msgs::Double & _msg){
  // Directly extract the double value from the message
  this->zVelocity = _msg.data();
  gzdbg << "Received new Z-velocity command: " << this->zVelocity << std::endl;
}

void MoveModelTopicWayPimp::Update(const UpdateInfo &_info,
                          EntityComponentManager &_ecm)
{
  // Only run if the simulation is not paused
  if (_info.paused)
    return;

  // 1. Find the target model entity by name (if not found yet)
  if (this->dataPtr->targetEntity == kNullEntity)
  {
    auto entityOpt = _ecm.EntityByName(this->dataPtr->modelName);
    if (!entityOpt.has_value())
    {
      gzdbg << "Model [" << this->dataPtr->modelName
            << "] not found yet. Skipping velocity application." << std::endl;
      return;
    }

    this->dataPtr->targetEntity = entityOpt.value();
    gzmsg << "Found target model entity: " << this->dataPtr->targetEntity << std::endl;
  }

  //method 1
  // 2.Get / create LinearVelocity component
  // auto velComp =
  //     _ecm.Component<components::LinearVelocityCmd>(this->dataPtr->targetEntity);

  // if (!velComp)
  // {
  //   velComp = _ecm.CreateComponent(this->dataPtr->targetEntity,
  //                                  components::LinearVelocityCmd());
  //   gzmsg << "Added LinearVelocity component to model: "
  //         << this->dataPtr->modelName << std::endl;
  // }

  // if (!velComp)
  // {
  //   gzerr << "Failed to create/get LinearVelocity component for model ["
  //         << this->dataPtr->modelName << "]." << std::endl;
  //   return;
  // }

  // // 3.Set the Z-axis linear velocity: (0, 0, zVelocity)
  // const gz::math::Vector3d vel(0.0, 0.0, this->dataPtr->zVelocity);
  // velComp->Data() = vel;

                  /// or
                  
  // method 2
  //instate of step 2 & 3 directly run     
  const gz::math::Vector3d vel(0.0, 0.0, this->dataPtr->zVelocity);     
  _ecm.SetComponentData<components::LinearVelocityCmd>(this->dataPtr->targetEntity,{vel});

}

// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::systems::MoveModelTopicWayPimp,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::MoveModelTopicWayPimp,
                    "gz::sim::systems::MoveModelTopicWayPimp")
