#include <rclcpp/rclcpp.hpp>
#include "pointcloud_to_laserscan_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_to_laserscan::PointCloudToLaserScanNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
