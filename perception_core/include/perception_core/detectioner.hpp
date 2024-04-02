
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"


namespace perception_core
{


class Detectioner
{
public:
  using Ptr = std::shared_ptr<perception_core::Detectioner>;


  virtual ~Detectioner() {}

  virtual void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
      std::string name, std::shared_ptr<tf2_ros::Buffer>) = 0;


  virtual void cleanup() = 0;


  virtual void activate() = 0;


  virtual void deactivate() = 0;

  

};

}










