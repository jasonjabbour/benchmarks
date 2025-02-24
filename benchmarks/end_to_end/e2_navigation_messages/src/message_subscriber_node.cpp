#include <rclcpp/rclcpp.hpp>
#include "message_subscriber.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robotperf::benchmark::MessageSubscriber>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}