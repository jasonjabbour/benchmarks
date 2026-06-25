#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace robotperf { namespace mapping {

class OccupancyGridOutputComponent : public rclcpp::Node
{
public:
  explicit OccupancyGridOutputComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  void cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  size_t get_msg_size(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
};

}} // namespace robotperf::mapping
