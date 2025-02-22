#include <rclcpp/rclcpp.hpp>
#include "pointcloud_input_component.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotperf::perception::PointCloudInputComponent>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
