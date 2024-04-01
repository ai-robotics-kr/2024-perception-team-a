#include <mutex>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace user_t{


    class Detection_t : public rclcpp::Node
    {
        typedef std::recursive_mutex mutex_t;
        
        public:
        explicit Detection_t(const rclcpp::NodeOptions &);


        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub;
        rclcpp::TimerBase::SharedPtr timer_;


        void timerCallback();
        void stringCallback(const std_msgs::msg::String::SharedPtr );


        void configure();
        void cleanup();
        void activate();
        void deactivate();
        void spinExecutor();




        mutex_t * getMutex()
        {
            return access_;
        }


        std::thread callback_group_executor_thread;
        rclcpp::executors::MultiThreadedExecutor callback_group_executor_;
        std_msgs::msg::String current_string;
        rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
        rclcpp::CallbackGroup::SharedPtr timer_cb_group_;


        private:



        protected:

        mutex_t * access_;



    };






}
