#ifndef LIGHT_CONTROL_ROS_PLUGIN_HH_
#define LIGHT_CONTROL_ROS_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Light.hh>
#include <gz/math/Color.hh>
#include <gz/sim/Light.hh>

#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>


#include <vector>

namespace gz
{
namespace sim
{
class LightControlDirectRos : public System, public ISystemPreUpdate, public ISystemConfigure
{
public:
  // Constructor
  LightControlDirectRos() = default;

  void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  // Function to find the light entity by name
  void FindLightEntities(EntityComponentManager &_ecm);

  //empty service call to turn on/off light
  void RosToggleCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Time accumulator for color cycling
  double time = 0.0;

  std::vector<Entity> lightEntities; //list of light entity

  gz::transport::Node node; //gz node
  std::string serviceName; //service name
  bool isLightOn; //val for light on/off
  bool pendingApply{false};   // apply state change once (e.g., when toggled OFF)


   // ROS 2 Members
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ros_srv_; 
  // rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_
  rclcpp::executors::SingleThreadedExecutor::SharedPtr ros_executor_; //Executor to spin the controller
  std::thread cpp_thread; //std thread to make new thread
};
}  // namespace sim
}  // namespace gz

#endif
