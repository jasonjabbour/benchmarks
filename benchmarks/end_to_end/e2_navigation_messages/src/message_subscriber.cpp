#include "message_subscriber.hpp"
#include <rclcpp/serialization.hpp>
#include <functional>

namespace robotperf
{
namespace benchmark
{

MessageSubscriber::MessageSubscriber(const rclcpp::NodeOptions & options)
: rclcpp::Node("message_subscriber", options)
{
    // Subscribe to standard message topics
    sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", 10, std::bind(&MessageSubscriber::pointcloud_callback, this, std::placeholders::_1));

    sub_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "laserscan", 10, std::bind(&MessageSubscriber::laserscan_callback, this, std::placeholders::_1));

    sub_occupancy_grid_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "occupancy_grid", 10, std::bind(&MessageSubscriber::occupancygrid_callback, this, std::placeholders::_1));

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", 10, std::bind(&MessageSubscriber::pose_callback, this, std::placeholders::_1));

    sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10, std::bind(&MessageSubscriber::twist_callback, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&MessageSubscriber::imu_callback, this, std::placeholders::_1));

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&MessageSubscriber::image_callback, this, std::placeholders::_1));

    // Subscribe to custom (quantized) message topics with distinct names.
    sub_pointcloud_custom_ = this->create_subscription<e3_custom_messages::msg::CustomPointCloud2>(
        "pointcloud_custom", 10, std::bind(&MessageSubscriber::pointcloud_custom_callback, this, std::placeholders::_1));

    sub_laserscan_custom_ = this->create_subscription<e3_custom_messages::msg::CustomLaserScan>(
        "laserscan_custom", 10, std::bind(&MessageSubscriber::laserscan_custom_callback, this, std::placeholders::_1));

    sub_pose_custom_ = this->create_subscription<e3_custom_messages::msg::CustomPoseWithCovarianceStamped>(
        "amcl_pose_custom", 10, std::bind(&MessageSubscriber::pose_custom_callback, this, std::placeholders::_1));

    sub_imu_custom_ = this->create_subscription<e3_custom_messages::msg::CustomImu>(
        "imu_custom", 10, std::bind(&MessageSubscriber::imu_custom_callback, this, std::placeholders::_1));
}

template<typename T>
size_t MessageSubscriber::get_msg_size(const T & msg)
{
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(&msg, &serialized_msg);
    return serialized_msg.size();
}

void MessageSubscriber::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);

    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

void MessageSubscriber::laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);

    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

void MessageSubscriber::occupancygrid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);

    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

void MessageSubscriber::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);
    
    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

void MessageSubscriber::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);
    
    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

void MessageSubscriber::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);
    
    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

void MessageSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);
    
    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

// NEW: Callback for custom (quantized) pointcloud messages
void MessageSubscriber::pointcloud_custom_callback(const e3_custom_messages::msg::CustomPointCloud2::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);
    
    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

// NEW: Callback for custom (quantized) laserscan messages
void MessageSubscriber::laserscan_custom_callback(const e3_custom_messages::msg::CustomLaserScan::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);
    
    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

// NEW: Callback for custom (quantized) pose messages
void MessageSubscriber::pose_custom_callback(const e3_custom_messages::msg::CustomPoseWithCovarianceStamped::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);
    
    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

// NEW: Callback for custom (quantized) IMU messages
void MessageSubscriber::imu_custom_callback(const e3_custom_messages::msg::CustomImu::SharedPtr msg)
{
    uint32_t unique_key = msg->header.stamp.nanosec;
    size_t msg_size = get_msg_size(*msg);
    
    TRACEPOINT(
        robotperf_msg_received_size_1,
        static_cast<const void *>(this),
        static_cast<const void *>(&(*msg)),
        unique_key,
        msg_size);
}

}  // namespace benchmark
}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::benchmark::MessageSubscriber)