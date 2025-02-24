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
    // else if (message_type_ == "laserscan")
    // {
    //     if (quantization_enabled_)
    //     {
    //         // Use e3_custom_messages::msg::CustomLaserScan when quantization is enabled
    //         e3_custom_messages::msg::CustomLaserScan msg;
    //         msg.header.stamp = this->now();
    //         msg.header.stamp.nanosec = unique_key;
    //         msg.angle_min = -1.57f;
    //         msg.angle_max = 1.57f;
    //         msg.angle_increment = 0.01f;
    //         msg.time_increment = 0.001f;
    //         msg.scan_time = 0.1f;
    //         msg.range_min = 0.1f;
    //         msg.range_max = 10.0f;
    //         msg.ranges.resize(message_size_, 5.0f);
    //         msg.intensities.resize(message_size_, 255.0f);
    //         size_t msg_size = get_msg_size(msg);
    //         TRACEPOINT(robotperf_msg_published_size_1,
    //                    static_cast<const void *>(this),
    //                    static_cast<const void *>(&msg),
    //                    unique_key, msg_size);
    //         pub_laserscan_custom_->publish(msg);
    //     }
    //     else
    //     {
    //         auto msg = sensor_msgs::msg::LaserScan();
    //         msg.header.stamp = this->now();
    //         msg.header.stamp.nanosec = unique_key;
    //         msg.angle_min = -1.57;
    //         msg.angle_max = 1.57;
    //         msg.angle_increment = 0.01;
    //         msg.time_increment = 0.001;
    //         msg.scan_time = 0.1;
    //         msg.range_min = 0.1;
    //         msg.range_max = 10.0;
    //         msg.ranges.resize(message_size_, 5.0);
    //         msg.intensities.resize(message_size_, 255);
    //         size_t msg_size = get_msg_size(msg);
    //         TRACEPOINT(robotperf_msg_published_size_1,
    //                    static_cast<const void *>(this),
    //                    static_cast<const void *>(&msg),
    //                    unique_key, msg_size);
    //         pub_laserscan_->publish(msg);
    //     }
    // }
    else if (message_type_ == "laserscan")
    {
        if (quantization_enabled_)
        {
            // -----------------------------------
            // Publish a custom (quantized) LaserScan
            // -----------------------------------
            using e3_custom_messages::msg::CustomLaserScan;

            CustomLaserScan msg;
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            msg.header.frame_id = "laser_frame";

            // Fill the angular metadata
            msg.angle_min = -1.57f;      // e.g., -90 deg
            msg.angle_max = 1.57f;       // +90 deg
            msg.angle_increment = 0.01f; // 100 points in 180 deg
            msg.time_increment = 0.0f;
            msg.scan_time = 0.0f;
            msg.range_min = 0.1f;
            msg.range_max = 50.0f;

            // Decide a scale factor for ranges. Suppose sensor range is up to 50 m,
            // and we want millimeter resolution => scale=1000 => 1 int16 step=0.001m
            // This means we can store up to 32.767 m if signed, or 65.535 m if we used uint16.
            // We'll do int16 for this example => max ~32.767 m unless we do special clamping.
            // If you want 50 m, you'd either clamp or use some offset. For simplicity, let's clamp.
            msg.range_scale = 1000.0f;

            // For intensities, assume we map from [0..255] => 1:1
            // or maybe we clamp real intensities from 0..1000 => 0..255
            // We'll pick 4x => 1 int step = 4.0 real intensity
            msg.intensity_scale = 4.0f;

            // We'll fill 'message_size_' data points
            // Note that we must allocate 'ranges' and 'intensities'
            msg.ranges.resize(message_size_);
            msg.intensities.resize(message_size_);

            for (size_t i = 0; i < message_size_; i++)
            {
            // Example "real" range in [0..50)
            float real_range = 5.0f + 0.01f * i;   // or wherever you get your data
            // Example "real" intensity in [0..1000)
            float real_intensity = 255.0f;        // or some dynamic value

            // Convert range to int16
            float scaled_range = real_range * msg.range_scale;
            // clamp to [-32768..32767], or just [0..32767] if ranges>0
            if (scaled_range < 0.0f) { 
                scaled_range = 0.0f; 
            } else if (scaled_range > 32767.0f) { 
                scaled_range = 32767.0f; 
            }
            int16_t stored_range = static_cast<int16_t>(std::round(scaled_range));

            // Convert intensity to uint8
            float scaled_intensity = real_intensity / msg.intensity_scale; // e.g. 255 => 63.75
            if (scaled_intensity < 0.0f) {
                scaled_intensity = 0.0f;
            } else if (scaled_intensity > 255.0f) {
                scaled_intensity = 255.0f;
            }
            uint8_t stored_intensity = static_cast<uint8_t>(std::round(scaled_intensity));

            msg.ranges[i] = stored_range;
            msg.intensities[i] = stored_intensity;
            }

            // Publish
            size_t msg_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                    static_cast<const void *>(this),
                    static_cast<const void *>(&msg),
                    unique_key, msg_size);

            pub_laserscan_custom_->publish(msg);
        }
        else {
            auto msg = sensor_msgs::msg::LaserScan();
            msg.ranges.resize(message_size_, 5.0);
            msg.intensities.resize(message_size_, 255);
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;

            size_t msg_size = get_msg_size(msg);

            TRACEPOINT(robotperf_msg_published_size_1, static_cast<const void *>(this), static_cast<const void *>(&msg), unique_key, msg_size);

            pub_laserscan_->publish(msg);
        }

    }
    else if (message_type_ == "pointcloud")
    {
        if (quantization_enabled_)
        {
            // -----------------------------------
            // Publish a *smaller* custom pointcloud
            // by using INT16 for x, y, z
            // -----------------------------------
            using e3_custom_messages::msg::CustomPointCloud2;
            using e3_custom_messages::msg::CustomPointField;

            CustomPointCloud2 msg;
            msg.header.stamp = this->now();
            msg.header.stamp.nanosec = unique_key;
            msg.header.frame_id = "velodyne";  // or any frame you need

            // For simplicity: single row
            msg.height = 1;
            msg.width = message_size_;

            // We'll store x, y, z => 3 fields
            msg.fields.resize(3);

            // Each field is INT16 = 2 bytes
            // x
            msg.fields[0].name = "x";
            msg.fields[0].offset = 0;
            msg.fields[0].datatype = CustomPointField::INT16;  // = 3
            msg.fields[0].count = 1;
            // y
            msg.fields[1].name = "y";
            msg.fields[1].offset = 2;
            msg.fields[1].datatype = CustomPointField::INT16;
            msg.fields[1].count = 1;
            // z
            msg.fields[2].name = "z";
            msg.fields[2].offset = 4;
            msg.fields[2].datatype = CustomPointField::INT16;
            msg.fields[2].count = 1;

            // Each point is 6 bytes total
            msg.point_step = 6;
            // For height=1, row_step = point_step * width
            msg.row_step = msg.point_step * msg.width;

            msg.is_bigendian = false;
            msg.is_dense = true;

            // Allocate data array: height × row_step
            msg.data.resize(msg.row_step * msg.height);

            // Example: fill x,y,z with some scaled values
            // We'll just do a scale factor of 100 => 1 int16 = 0.01
            const float scale = 100.0f; 

            // We'll reinterpret the data buffer as int16 pointer
            int16_t* data_ptr = reinterpret_cast<int16_t*>(msg.data.data());

            for (size_t i = 0; i < msg.width; i++)
            {
            // Some example "real" coordinates in [0..10)
            float x_float = static_cast<float>(i) / 50.0f;  // or any pattern
            float y_float = 1.2345f;
            float z_float = (static_cast<float>(i) / 50.0f) + 2.0f;

            // Convert to int16 with chosen scale
            int16_t x_int = static_cast<int16_t>(std::round(x_float * scale));
            int16_t y_int = static_cast<int16_t>(std::round(y_float * scale));
            int16_t z_int = static_cast<int16_t>(std::round(z_float * scale));

            // Store them
            data_ptr[3*i + 0] = x_int;  // offset 0
            data_ptr[3*i + 1] = y_int;  // offset 2
            data_ptr[3*i + 2] = z_int;  // offset 4
            }

            // Now measure and publish
            size_t serialized_size = get_msg_size(msg);
            TRACEPOINT(robotperf_msg_published_size_1,
                    static_cast<const void *>(this),
                    static_cast<const void *>(&msg),
                    unique_key, serialized_size);

            pub_pointcloud_custom_->publish(msg);
        }
        else {
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