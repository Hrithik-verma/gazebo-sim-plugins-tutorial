#include "tutorial_gazebo_plugins/MoveModelDirectRos.hh"

#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/Name.hh"
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

#include <rclcpp/executors.hpp> // New: ROS 2 includes


using namespace gz;
using namespace sim;
using namespace systems;

MoveModelDirectRos::MoveModelDirectRos(){
  std::cout<<"MoveModelDirectRos Plugin Started!!"<<std::endl;

}
MoveModelDirectRos::~MoveModelDirectRos() {
  std::cout<<"MoveModelDirectRos Plugin Stopped!!"<<std::endl;

}

void MoveModelDirectRos::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // --- Gazebo Sim Configuration ---
  if (!_sdf->HasElement("model_name"))
  {
    gzerr << "MoveModel plugin requires a <model_name> element." << std::endl;
    return;
  }
  this->modelName = _sdf->Get<std::string>("model_name");
  gzmsg << "Target Model Name: " << this->modelName << std::endl;

  if (!_sdf->HasElement("ros_topic_name"))
  {
    gzerr << "MoveModel plugin requires a <topic_name> element." << std::endl;
    return;
  }

  this->rosTopicName = _sdf->Get<std::string>("ros_topic_name");
  gzmsg << "Cmd vel Z ROS 2 Topic Name: " << this->rosTopicName << std::endl;
  
  if (_sdf->HasElement("z_velocity"))
  {
    this->zVelocity = _sdf->Get<double>("z_velocity");
  }
  gzmsg << "Initial Z Velocity: " << this->zVelocity << " m/s" << std::endl;


  // --- ROS 2 Initialization ---
  // Initialize ROS 2 (safe to call multiple times)
  if (!rclcpp::ok())
  {
    // Pass in dummy arguments.
    rclcpp::init(0, nullptr); 
  }

  // ### Explanation ###
  /*

  ros executor code snap:
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // blocks this thread, one callback at a time

  gz main thread:
  ------

  in loop:
   PreUpdate()     -> using ros spin (executor.spin()) in main thread  will block gazebo sim
   Update()       
   PostUpdate()

   
   solution:

   main gz thread                  --->(make another thread for ros)  using std::thread 
   ------                           |
                                    |    -> ros spin work with blocking but not harm gz main thread 
  in loop:                          |        as it a different thread
   PreUpdate()    -> no blocking    |    
   Update()                         |    -> std::thread -> make new thread cpp_thread having ros_executor->spin()
   PostUpdate()                     |  

  */

  // Create the ROS 2 node
  this->ros_node_ = std::make_shared<rclcpp::Node>("gz_sim_move_model_plugin");
//   this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->ros_executor_->add_node(this->ros_node_);

  //lamda function having ros spin
  auto ros_thread_spin = [this]() 
  { 
    this->ros_executor_->spin(); 
  };

  //make a new thread for ros spin
  this->cpp_thread = std::thread(ros_thread_spin);


  // Create the ROS 2 subscriber
  this->sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float64>(
      this->rosTopicName, 
      10, // QoS history depth
      std::bind(&MoveModelDirectRos::OnRosMsg, this, std::placeholders::_1)
  );
  
  gzmsg << "ROS 2 Subscriber created for topic: " << this->rosTopicName << std::endl;
}


void MoveModelDirectRos::OnRosMsg(const std_msgs::msg::Float64::SharedPtr _msg){
  // Extract the double value from the ROS 2 Float64 message
  this->zVelocity = _msg->data;
  gzdbg << "Received new Z-velocity command from ROS 2: " << this->zVelocity << std::endl;
}

void MoveModelDirectRos::Update(const UpdateInfo &_info,
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

  // 2. Set the Z-axis linear velocity: (0, 0, zVelocity)
  const gz::math::Vector3d vel(0.0, 0.0, this->zVelocity);

  // 3. Apply the velocity command component
  _ecm.SetComponentData<components::LinearVelocityCmd>(this->targetEntity,{vel});
}

// Register the plugin with Gazebo Sim
GZ_ADD_PLUGIN(gz::sim::systems::MoveModelDirectRos,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::MoveModelDirectRos,
                    "gz::sim::systems::MoveModelDirectRos")