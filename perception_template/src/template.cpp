#include "perception_template/template.hpp"


using namespace std::chrono_literals;


namespace user_t {

    
  Detection_t::Detection_t(const rclcpp::NodeOptions & options)
  :Node ("user_template_checknode",options)
  {
    RCLCPP_INFO(get_logger(),"node starting");

    // Detection_t::configure();

    // Detection_t::activate();
  
  }



 void Detection_t::configure(){

  subscriber_cb_group_ =create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  // executor.add_callback_group(callback_group_, get_node_base_interface());
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group= subscriber_cb_group_;


   string_sub = create_subscription<std_msgs::msg::String>(
            "/example",rclcpp::SystemDefaultsQoS(),
        std::bind(&Detection_t::stringCallback,this,std::placeholders::_1),sub_options);



    callback_group_executor_.add_callback_group(subscriber_cb_group_, this->get_node_base_interface());
    callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});





  timer_ = this->create_wall_timer(100ms,std::bind(&Detection_t::timerCallback,this));


    
  }



  void Detection_t::timerCallback(){}

  void Detection_t::cleanup(){}

  void Detection_t::activate(){}

  void Detection_t::deactivate(){}




  void Detection_t::stringCallback(const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(get_logger(),"sub");
            current_string = *msg; 
    }




}


