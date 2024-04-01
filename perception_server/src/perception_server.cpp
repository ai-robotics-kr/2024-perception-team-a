#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "perception_server/perception_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace perception_server
{

PerceptionServer::PerceptionServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("perception_server", "", options),
    default_ids_{"YoloV8", "BEVfusion", "OCR"},
    lp_loader_("perception_core", "percception_core::Detectioner")
{
  RCLCPP_INFO(get_logger(), "Creating");
  // Declare this node's parameters
  declare_parameter("Detection_plugins", default_ids_);
  

  // Setup the perception_costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "detection_costmap", std::string{get_namespace()}, "detection_costmap");
}



void
PerceptionServer::GetParameters(){

  auto node = shared_from_this();
  get_parameter("Detection_plugins", detection_plugins_);

  //for debugging///////////////////////////////////////////
  for(size_t i=0; i< detection_plugins_.size(); i++)
  {RCLCPP_INFO(get_logger(),"detection_plugins : {%s}",detection_plugins_[i].c_str());}
  //////////////////////////////////////////////////////////

  if (detection_plugins_.size() ==0){RCLCPP_ERROR(get_logger(),"params error");}

  // if (detection_plugins_ == default_ids_) {
  //   for (size_t i = 0; i < default_ids_.size(); ++i) {
  //     nav2_util::declare_parameter_if_not_declared(
  //       node, default_ids_[i] + ".plugin",
  //       rclcpp::ParameterValue("1"));
  //   }
  // }
  else {
    for (size_t i = 0; i < detection_plugins_.size(); ++i) {
        nav2_util::declare_parameter_if_not_declared(
        node, detection_plugins_[i] + ".sources", rclcpp::ParameterValue("camera"));

        nav2_util::declare_parameter_if_not_declared(
        node, detection_plugins_[i] + ".plugin", rclcpp::ParameterValue("."));
    }
    }





}



PerceptionServer::~PerceptionServer()
{

  // costmap_thread_.reset();
}

nav2_util::CallbackReturn
PerceptionServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();
  RCLCPP_INFO(get_logger(), "Configuring");
  GetParameters();



    costmap_ros_->configure();
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  
  for (size_t i = 0; i != detection_plugins_.size(); i++) {
    try {
      detection_types_[i] = nav2_util::get_plugin_type_param(node, detection_plugins_[i]);

      RCLCPP_INFO(get_logger(),"now");
      perception_core::Detectioner::Ptr detectioner =
        lp_loader_.createUniqueInstance(detection_types_[i]);

      RCLCPP_INFO(
        get_logger(), "Created detectioner : %s of type %s",
        detection_plugins_[i].c_str(), detection_types_[i].c_str());
      detectioner->configure(node, detection_plugins_[i],costmap_ros_->getTfBuffer());
      detectioners_.insert({detection_plugins_[i], detectioner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create controller. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }


  detection_types_.resize(detection_plugins_.size());





  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PerceptionServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");


  costmap_ros_->activate();



  auto node = shared_from_this();



  // Add callback for dynamic parameters
  // dyn_params_handler_ = node->add_on_set_parameters_callback(
  //   std::bind(&PerceptionServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PerceptionServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");


  // if (costmap_ros_->get_current_state().id() ==
  //   lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  // {
  //   costmap_ros_->deactivate();
  // }


  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PerceptionServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  tf_buffer_.reset();


  // if (costmap_ros_->get_current_state().id() ==
  //   lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  // {
  //   costmap_ros_->cleanup();
  // }

  // costmap_thread_.reset();
  // costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PerceptionServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}


void PerceptionServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}


// rcl_interfaces::msg::SetParametersResult
// PerceptionServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
// {
//   std::lock_guard<std::mutex> lock(dynamic_params_lock_);
//   rcl_interfaces::msg::SetParametersResult result;

//   for (auto parameter : parameters) {
//     const auto & type = parameter.get_type();
//     const auto & name = parameter.get_name();

//   }

//   result.successful = true;
//   return result;
// }

}  // namespace perception_server

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(perception_server::PerceptionServer)
