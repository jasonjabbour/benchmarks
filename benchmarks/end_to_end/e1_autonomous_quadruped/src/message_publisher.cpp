#include "message_publisher.hpp"
#include <rclcpp/serialization.hpp>

namespace robotperf
{
namespace benchmark
{

MessagePublisher::MessagePublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("message_publisher", options)
{
    // Declare parameters
    message_type_ = this->declare_parameter<std::string>("message_type", "pointcloud");
    message_size_ = this->declare_parameter<int>("message_size", 1000);  // Default 1000 elements

    // Initialize publishers for different message types
    pub_occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
    pub_laserscan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laserscan", 10);
    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10);
    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

    // Timer to publish messages at 10 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MessagePublisher::publish_message, this)
    );
}

uint32_t MessagePublisher::generate_unique_key()
{
    static uint32_t counter = 1;
    return counter++;
}

template<typename T>
size_t MessagePublisher::get_msg_size(const T & msg)
{
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(&msg, &serialized_msg);
    return serialized_msg.size();
}

void MessagePublisher::publish_message()
{
    uint32_t unique_key = generate_unique_key();

    if (message_type_ == "occupancy_grid")
    {
        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.info.width = std::sqrt(message_size_);
        msg.info.height = std::sqrt(message_size_);
        msg.data.resize(message_size_, 100);  // Filled with arbitrary values
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;

        size_t msg_size = get_msg_size(msg);

        TRACEPOINT(robotperf_msg_published_size_1, static_cast<const void *>(this), static_cast<const void *>(&msg), unique_key, msg_size);

        pub_occupancy_grid_->publish(msg);
    }
    else if (message_type_ == "laserscan")
    {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.ranges.resize(message_size_, 5.0);
        msg.intensities.resize(message_size_, 255);
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;

        size_t msg_size = get_msg_size(msg);

        TRACEPOINT(robotperf_msg_published_size_1, static_cast<const void *>(this), static_cast<const void *>(&msg), unique_key, msg_size);

        pub_laserscan_->publish(msg);
    }
    else if (message_type_ == "pointcloud")
    {
        auto msg = sensor_msgs::msg::PointCloud2();
        msg.height = 1;
        msg.width = message_size_;
        msg.fields.resize(3);  // x, y, z fields
        msg.data.resize(message_size_ * 12, 0);
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;

        size_t msg_size = get_msg_size(msg);

        TRACEPOINT(robotperf_msg_published_size_1, static_cast<const void *>(this), static_cast<const void *>(&msg), unique_key, msg_size);
            
        pub_pointcloud_->publish(msg);
    }
    else if (message_type_ == "amcl_pose")
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;
        msg.pose.pose.position.x = 1.0;
        msg.pose.pose.orientation.w = 1.0;

        size_t msg_size = get_msg_size(msg);

        TRACEPOINT(robotperf_msg_published_size_1, static_cast<const void *>(this), static_cast<const void *>(&msg), unique_key, msg_size);

        pub_pose_->publish(msg);
    }
    else if (message_type_ == "twist_stamped")
    {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;
        msg.twist.linear.x = 1.0;
        msg.twist.angular.z = 0.5;

        size_t msg_size = get_msg_size(msg);

        TRACEPOINT(robotperf_msg_published_size_1, static_cast<const void *>(this), static_cast<const void *>(&msg), unique_key, msg_size);

        pub_twist_->publish(msg);
    }
    else if (message_type_ == "imu")
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;
        msg.orientation.w = 1.0;  
        msg.angular_velocity.x = 0.1;
        msg.linear_acceleration.z = 9.81;  

        for (int i = 0; i < 9; i++) {
            msg.orientation_covariance[i] = 0.1;
            msg.angular_velocity_covariance[i] = 0.1;
            msg.linear_acceleration_covariance[i] = 0.1;
        }

        size_t msg_size = get_msg_size(msg);

        TRACEPOINT(robotperf_msg_published_size_1, static_cast<const void *>(this), static_cast<const void *>(&msg), unique_key, msg_size);

        pub_imu_->publish(msg);
    }
    else if (message_type_ == "image")
    {
        auto msg = sensor_msgs::msg::Image();
        msg.height = std::sqrt(message_size_);
        msg.width = std::sqrt(message_size_);
        msg.encoding = "rgb8"; 
        msg.step = msg.width * 3; 
        msg.data.resize(msg.height * msg.step, 255);  
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;

        size_t msg_size = get_msg_size(msg);

        TRACEPOINT(robotperf_msg_published_size_1, static_cast<const void *>(this), static_cast<const void *>(&msg), unique_key, msg_size);

        pub_image_->publish(msg);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unsupported message type: %s", message_type_.c_str());
    }
}

}  // namespace benchmark
}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::benchmark::MessagePublisher)
