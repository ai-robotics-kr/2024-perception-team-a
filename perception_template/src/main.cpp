#include <memory>

#include "perception_template/template.hpp"


int main (int argc, char ** argv){
    rclcpp::init(argc,argv);
    const rclcpp::NodeOptions options;
    auto node = std::make_shared<user_t::Detection_t>(options);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();


    return 0;


}