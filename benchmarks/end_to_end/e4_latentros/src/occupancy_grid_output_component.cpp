#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "tracetools_benchmark/tracetools.h"
#include "e4_latentros/occupancy_grid_output_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf { namespace mapping {

OccupancyGridOutputComponent::OccupancyGridOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("OccupancyGridOutputComponent", options)
{
  auto topic = declare_parameter<std::string>("output_topic_name", "/global_costmap/costmap");
  // Match costmap publisher QoS: RELIABLE + TRANSIENT_LOCAL
  sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::QoS(10).reliable().transient_local().transient_local(),
    std::bind(&OccupancyGridOutputComponent::cb, this, std::placeholders::_1));
}

size_t OccupancyGridOutputComponent::get_msg_size(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  rclcpp::SerializedMessage s;
  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> ser;
  ser.serialize_message(msg.get(), &s);
  return s.size();
}

void OccupancyGridOutputComponent::cb(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  uint32_t key = msg->header.stamp.nanosec;

  TRACEPOINT(robotperf_occupancy_grid_output_cb_init,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);

  TRACEPOINT(robotperf_occupancy_grid_output_cb_fini,
    static_cast<const void *>(this), static_cast<const void *>(&(*msg)),
    msg->header.stamp.nanosec, msg->header.stamp.sec, get_msg_size(msg), key);
}

}} // namespace

RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::mapping::OccupancyGridOutputComponent)
