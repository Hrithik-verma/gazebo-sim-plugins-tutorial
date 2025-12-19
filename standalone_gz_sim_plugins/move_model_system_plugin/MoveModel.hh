#ifndef SYSTEM_PLUGIN_MODEL_HH_
#define SYSTEM_PLUGIN_MODEL_HH_

//! [header]
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
namespace systems
{
  /// \brief plugin to move a model
  /// plugin interface.
  class MoveModel :
    // This class is a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemPreUpdate
  {
   public:
    MoveModel();

    ~MoveModel() override;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    void PreUpdate(const UpdateInfo &_info,
                  EntityComponentManager &_ecm) override;

   private:
    std::string modelName;
    double zVelocity{0.0};
    Entity targetEntity{kNullEntity};
  };
}
}
}

//! [header]

#endif
