/**
 * LatentROS: Savitzky-Golay path smoother benchmark.
 * Pure topic-driven: Path in → smooth → Path out
 * Implements Savitzky-Golay smoothing directly (no Nav2 plugin dependency).
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include "tracetools_benchmark/tracetools.h"

class SmootherBenchmarkNode : public rclcpp::Node
{
public:
  SmootherBenchmarkNode()
  : rclcpp::Node("smoother_benchmark_node")
  {
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/robotperf/benchmark/path_input",
      rclcpp::QoS(10).reliable(),
      std::bind(&SmootherBenchmarkNode::pathCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/benchmark/smoothed_path",
      rclcpp::QoS(10).reliable());

    RCLCPP_INFO(get_logger(), "Savitzky-Golay smoother benchmark node started");
  }

private:
  void pathCallback(nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 5) return;

    uint32_t key = msg->header.stamp.nanosec;
    TRACEPOINT(robotperf_msg_received_1,
      static_cast<const void *>(this),
      static_cast<const void *>(msg.get()), key);

    // Savitzky-Golay smoothing (window=5, polynomial order 2)
    // Coefficients: [-3, 12, 17, 12, -3] / 35
    nav_msgs::msg::Path smoothed = *msg;
    size_t n = smoothed.poses.size();
    for (int pass = 0; pass < 2; pass++) {
      auto input = smoothed;
      for (size_t i = 2; i < n - 2; i++) {
        smoothed.poses[i].pose.position.x =
          (-3.0 * input.poses[i-2].pose.position.x +
           12.0 * input.poses[i-1].pose.position.x +
           17.0 * input.poses[i].pose.position.x +
           12.0 * input.poses[i+1].pose.position.x +
           -3.0 * input.poses[i+2].pose.position.x) / 35.0;
        smoothed.poses[i].pose.position.y =
          (-3.0 * input.poses[i-2].pose.position.y +
           12.0 * input.poses[i-1].pose.position.y +
           17.0 * input.poses[i].pose.position.y +
           12.0 * input.poses[i+1].pose.position.y +
           -3.0 * input.poses[i+2].pose.position.y) / 35.0;
      }
    }

    smoothed.header.stamp.nanosec = key;

    TRACEPOINT(robotperf_msg_published_1,
      static_cast<const void *>(this),
      static_cast<const void *>(&smoothed), key);
    path_pub_->publish(smoothed);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmootherBenchmarkNode>());
  rclcpp::shutdown();
  return 0;
}
