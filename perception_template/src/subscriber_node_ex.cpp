#include "perception_template/subscriber_node_ex.hpp"


using namespace std::chrono_literals;


namespace user_t{

    subscriber_node::subscriber_node()
    : Node("subscriber_node_example")
    {

        string_sub = create_subscription<std_msgs::msg::String>(
            "example",rclcpp::SystemDefaultsQoS(),
        std::bind(&subscriber_node::stringCallback,this,std::placeholders::_1));

       timer_ = this->create_wall_timer(20ms,std::bind(&subscriber_node::timerCallback,this));
        RCLCPP_INFO(get_logger(),"now now now");

    }

    void subscriber_node::timerCallback(){

        RCLCPP_INFO(get_logger(),"timer callback");
        tick ++;

    }


    void subscriber_node::stringCallback(const std_msgs::msg::String::SharedPtr msg){

           current_string = *msg; 
            RCLCPP_INFO(get_logger(),"sub");
    }

    int subscriber_node::get_tick(){

        return tick;
        
    }

    std_msgs::msg::String subscriber_node::get_string(){

    return current_string;

    }



















}