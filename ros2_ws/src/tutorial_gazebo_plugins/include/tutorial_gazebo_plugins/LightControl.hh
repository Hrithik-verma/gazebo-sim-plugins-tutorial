#ifndef LIGHT_CONTROL_PLUGIN_HH_
#define LIGHT_CONTROL_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Light.hh>
#include <gz/math/Color.hh>
#include <gz/sim/Light.hh>

#include <vector>

namespace gz
{
namespace sim
{
class LightControl : public System, public ISystemPreUpdate
{
public:
  // Constructor
  LightControl() = default;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  // Function to find the light entity by name
  void FindLightEntities(EntityComponentManager &_ecm);

  // Time accumulator for color cycling
  double time = 0.0;

  std::vector<Entity> lightEntites; //list of light entity
};
}  // namespace sim
}  // namespace gz

#endif
