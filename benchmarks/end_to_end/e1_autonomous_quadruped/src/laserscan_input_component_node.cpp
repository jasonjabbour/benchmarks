#include <rclcpp/rclcpp.hpp>
#include "laserscan_input_component.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotperf::perception::LaserscanInputComponent>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}