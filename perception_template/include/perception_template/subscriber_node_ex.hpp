#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
namespace user_t
{


    class subscriber_node : public rclcpp::Node
    {

        public:
            // subscriber_node() : Node()

        // explicit subscriber_node(const rclcpp::NodeOptions &);
        explicit subscriber_node();
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub;
        rclcpp::TimerBase::SharedPtr timer_;


        void timerCallback();
        void stringCallback(const std_msgs::msg::String::SharedPtr );

        std_msgs::msg::String get_string();
        int get_tick();

        std_msgs::msg::String current_string;
        int tick;
        
        private:

        protected:


    };







}