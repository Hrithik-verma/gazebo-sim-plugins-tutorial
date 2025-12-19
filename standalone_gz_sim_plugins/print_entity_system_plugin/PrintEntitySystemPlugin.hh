#ifndef WORLD_PLUGIN_HH
#define WORLD_PLUGIN_HH

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
  class PrintEntitySystemPlugin :
    // This class is a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemPreUpdate
  {
   public:
    PrintEntitySystemPlugin();

    ~PrintEntitySystemPlugin() override;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    void PreUpdate(const UpdateInfo &_info,
                  EntityComponentManager &_ecm) override;

   private:
    std::string modelName;
    Entity targetEntity;
  };
}
}
}

//! [header]

#endif
