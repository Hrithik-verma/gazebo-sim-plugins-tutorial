#ifndef SYSTEM_MOVE_MODEL_DIRECTROS_PLUGIN_MODEL_HH_
#define SYSTEM_MOVE_MODEL_DIRECTROS_PLUGIN_MODEL_HH_

//! [header]
#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>

// ROS 2 Includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gz
{
namespace sim
{
namespace systems
{
  /// \brief plugin to move a model
  /// plugin interface.
  class MoveModelDirectRos :
    // This class is a system.
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemUpdate
  {
   public:
    MoveModelDirectRos();

    ~MoveModelDirectRos() override;

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    void Update(const UpdateInfo &_info,
                  EntityComponentManager &_ecm) override;

   private:
    std::string modelName;
    std::string rosTopicName;
    double zVelocity{0.0};
    Entity targetEntity{kNullEntity};
    // ROS 2 Members
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    // rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_
    rclcpp::executors::SingleThreadedExecutor::SharedPtr ros_executor_; //Executor to spin the controller
    std::thread cpp_thread; //std thread to make new thread


    /* ROS 2 message callback */
    /**
     * @brief ROS 2 callback function for received velocity messages.
     * @param _msg The shared pointer to the incoming std_msgs/Float64 message.
    */
    void OnRosMsg(const std_msgs::msg::Float64::SharedPtr _msg);
  };
}
}
}

//! [header]

#endif
