#ifndef SYSTEM_PLUGIN_MODEL_HH_
#define SYSTEM_PLUGIN_MODEL_HH_

//! [header]
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh> //for gz transport node
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h> // New: Include the Double message type

namespace gz
{
namespace sim
{
namespace systems
{
  /// \brief plugin to move a model
  /// plugin interface.
  class MoveModelTopicWay :
    // This class is a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemUpdate
  {
   public:
    MoveModelTopicWay();

    ~MoveModelTopicWay() override;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    void Update(const UpdateInfo &_info,
                  EntityComponentManager &_ecm) override;

   private:
    std::string modelName;
    std::string topicName;
    double zVelocity{0.0};
    Entity targetEntity{kNullEntity};
    gz::transport::Node node; // GZ Transport node

    /* transport msg callback */
    void OnTransportMsg(const gz::msgs::Double & _msg);
  };
}
}
}

//! [header]

#endif
