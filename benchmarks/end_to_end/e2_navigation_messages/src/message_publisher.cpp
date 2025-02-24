#include "message_publisher.hpp"
#include <rclcpp/serialization.hpp>
#include <random>
#include <cstring>

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
    quantization_enabled_ = this->declare_parameter<bool>("quantization_enabled", false);  // Default: Off

    // Initialize publishers for different message types

    pub_occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);

    // For topics that support quantization, create two publishers with different topic names.
    pub_laserscan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laserscan", 10);
    pub_laserscan_custom_ = this->create_publisher<e3_custom_messages::msg::CustomLaserScan>("laserscan_custom", 10);

    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    pub_pointcloud_custom_ = this->create_publisher<e3_custom_messages::msg::CustomPointCloud2>("pointcloud_custom", 10);

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10);
    pub_pose_custom_ = this->create_publisher<e3_custom_messages::msg::CustomPoseWithCovarianceStamped>("amcl_pose_custom", 10);

    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    pub_imu_custom_ = this->create_publisher<e3_custom_messages::msg::CustomImu>("imu_custom", 10);

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

// Convert float64 to IEEE 754 float16 (legacy conversion function)
uint16_t MessagePublisher::float64_to_float16(double f)
{
    uint64_t* f_as_int = reinterpret_cast<uint64_t*>(&f);
    uint16_t sign = (*f_as_int >> 48) & 0x8000;
    uint16_t exponent = ((*f_as_int >> 52) & 0x7FF) - 1023 + 15;
    uint16_t mantissa = (*f_as_int >> 42) & 0x03FF;  // Take 10 bits
    return sign | (exponent << 10) | mantissa;
}

float quantize_float32(float value, int scale_factor = 1024)
{
    return round(value * scale_factor) / scale_factor;
}

void MessagePublisher::publish_message()
{
    uint32_t unique_key = generate_unique_key();

    if (message_type_ == "occupancy_grid")
    {
        if (quantization_enabled_) {
            RCLCPP_INFO(this->get_logger(), "Quantization is not applicable to occupancy_grid.");
        }

        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.info.width = std::sqrt(message_size_);
        msg.info.height = std::sqrt(message_size_);
        msg.data.resize(message_size_, 100);
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;

        size_t msg_size = get_msg_size(msg);
        TRACEPOINT(robotperf_msg_published_size_1,
                   static_cast<const void *>(this),
                   static_cast<const void *>(&msg),
                   unique_key, msg_size);

        pub_occupancy_grid_->publish(msg);
    }
    else if (message_type_ == "laserscan")
    {
        if (quantization_enabled_)
        {
            // Use e3_custom_messages::msg::CustomLaserScan when quantization is enabled
            e3_custom_messages::msg::CustomLaserScan msg;
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            msg.angle_min = -1.57f;
            msg.angle_max = 1.57f;
            msg.angle_increment = 0.01f;
            msg.time_increment = 0.001f;
            msg.scan_time = 0.1f;
            msg.range_min = 0.1f;
            msg.range_max = 10.0f;
            msg.ranges.resize(message_size_, 5.0f);
            msg.intensities.resize(message_size_, 255.0f);
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&msg),
                       unique_key, msg_size);
            pub_laserscan_custom_->publish(msg);
        }
        else
        {
            auto msg = sensor_msgs::msg::LaserScan();
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            msg.angle_min = -1.57;
            msg.angle_max = 1.57;
            msg.angle_increment = 0.01;
            msg.time_increment = 0.001;
            msg.scan_time = 0.1;
            msg.range_min = 0.1;
            msg.range_max = 10.0;
            msg.ranges.resize(message_size_, 5.0);
            msg.intensities.resize(message_size_, 255);
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&msg),
                       unique_key, msg_size);
            pub_laserscan_->publish(msg);
        }
    }
    else if (message_type_ == "pointcloud")
    {
        if (quantization_enabled_)
        {
            // Use e3_custom_messages::msg::CustomPointCloud2 for quantized pointcloud
            e3_custom_messages::msg::CustomPointCloud2 msg;
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            msg.height = 1;
            msg.width = message_size_;
            msg.fields.resize(3);
            msg.data.resize(message_size_ * 12, 0);
            // Optionally: apply quantization logic to the data fields here
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&msg),
                       unique_key, msg_size);
            pub_pointcloud_custom_->publish(msg);
        }
        else
        {
            auto msg = sensor_msgs::msg::PointCloud2();
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            msg.height = 1;
            msg.width = message_size_;
            msg.fields.resize(3);
            msg.data.resize(message_size_ * 12, 0);
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&msg),
                       unique_key, msg_size);
            pub_pointcloud_->publish(msg);
        }
    }
    else if (message_type_ == "amcl_pose")
    {
        if (quantization_enabled_)
        {
            // Use e3_custom_messages::msg::CustomPoseWithCovarianceStamped
            e3_custom_messages::msg::CustomPoseWithCovarianceStamped msg;
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            // Fill in with quantized values (float32)
            msg.pose.pose.position.x = 1.0f;
            msg.pose.pose.orientation.w = 1.0f;
            for (int i = 0; i < 36; i++)
            {
                msg.pose.covariance[i] = (i % 4 == 0) ? 0.1f : 0.0f;
            }
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&msg),
                       unique_key, msg_size);
            pub_pose_custom_->publish(msg);
        }
        else
        {
            auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            msg.pose.pose.position.x = 1.0;
            msg.pose.pose.orientation.w = 1.0;
            for (int i = 0; i < 36; i++)
            {
                msg.pose.covariance[i] = (i % 4 == 0) ? 0.1 : 0.0;
            }
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&msg),
                       unique_key, msg_size);
            pub_pose_->publish(msg);
        }
    }
    else if (message_type_ == "twist_stamped")
    {
        if (quantization_enabled_) {
            RCLCPP_INFO(this->get_logger(), "Quantization is not applicable to twist_stamped.");
        }
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;
        msg.twist.linear.x = 1.0;
        msg.twist.angular.z = 0.5;
        size_t msg_size = get_msg_size(msg);
        TRACEPOINT(robotperf_msg_published_size_1,
                   static_cast<const void *>(this),
                   static_cast<const void *>(&msg),
                   unique_key, msg_size);
        pub_twist_->publish(msg);
    }
    else if (message_type_ == "imu")
    {
        if (quantization_enabled_)
        {
            // Use e3_custom_messages::msg::CustomImu for quantized IMU messages
            e3_custom_messages::msg::CustomImu msg;
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> noise(-0.05, 0.05);
            msg.orientation.x = static_cast<float>(noise(gen));
            msg.orientation.y = static_cast<float>(noise(gen));
            msg.orientation.z = static_cast<float>(noise(gen));
            msg.orientation.w = static_cast<float>(1.0 + noise(gen));
            msg.angular_velocity.x = static_cast<float>(0.1 + noise(gen));
            msg.angular_velocity.y = static_cast<float>(-0.1 + noise(gen));
            msg.angular_velocity.z = static_cast<float>(0.05 + noise(gen));
            msg.linear_acceleration.x = static_cast<float>(noise(gen));
            msg.linear_acceleration.y = static_cast<float>(noise(gen));
            msg.linear_acceleration.z = static_cast<float>(9.81 + noise(gen));
            for (int i = 0; i < 9; i++) {
                msg.orientation_covariance[i] = static_cast<float>((i % 4 == 0) ? (0.1 + noise(gen)) : noise(gen));
                msg.angular_velocity_covariance[i] = static_cast<float>((i % 4 == 0) ? (0.1 + noise(gen)) : noise(gen));
                msg.linear_acceleration_covariance[i] = static_cast<float>((i % 4 == 0) ? (0.1 + noise(gen)) : noise(gen));
            }
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&msg),
                       unique_key, msg_size);
            pub_imu_custom_->publish(msg);
        }
        else
        {
            auto msg = sensor_msgs::msg::Imu();
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> noise(-0.05, 0.05);
            msg.orientation.x = noise(gen);
            msg.orientation.y = noise(gen);
            msg.orientation.z = noise(gen);
            msg.orientation.w = 1.0 + noise(gen);
            msg.angular_velocity.x = 0.1 + noise(gen);
            msg.angular_velocity.y = -0.1 + noise(gen);
            msg.angular_velocity.z = 0.05 + noise(gen);
            msg.linear_acceleration.x = noise(gen);
            msg.linear_acceleration.y = noise(gen);
            msg.linear_acceleration.z = 9.81 + noise(gen);
            for (int i = 0; i < 9; i++) {
                msg.orientation_covariance[i] = (i % 4 == 0) ? 0.1 + noise(gen) : noise(gen);
                msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.1 + noise(gen) : noise(gen);
                msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.1 + noise(gen) : noise(gen);
            }
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                       static_cast<const void *>(this),
                       static_cast<const void *>(&msg),
                       unique_key, msg_size);
            pub_imu_->publish(msg);
        }
    }
    else if (message_type_ == "image")
    {
        if (quantization_enabled_) {
            RCLCPP_INFO(this->get_logger(), "Quantization is not applicable to image.");
        }
        auto msg = sensor_msgs::msg::Image();
        msg.height = std::sqrt(message_size_);
        msg.width = std::sqrt(message_size_);
        msg.encoding = "rgb8";
        msg.step = msg.width * 3;
        msg.data.resize(msg.height * msg.step, 255);
        msg.header.stamp = this->now();
        msg.header.stamp.nanosec = unique_key;
        size_t msg_size = get_msg_size(msg);
        TRACEPOINT(robotperf_msg_published_size_1,
                   static_cast<const void *>(this),
                   static_cast<const void *>(&msg),
                   unique_key, msg_size);
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