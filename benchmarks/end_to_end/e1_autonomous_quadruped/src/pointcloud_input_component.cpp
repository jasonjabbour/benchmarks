#include <rclcpp/rclcpp.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "pointcloud_input_component.hpp"
#include <rclcpp/serialization.hpp>

namespace robotperf
{

namespace perception
{

PointCloudInputComponent::PointCloudInputComponent (const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudInputComponent", options)
{

  // Parameter to enable or disable quantization
  quantization_enabled_ = this->declare_parameter<bool>("quantization_enabled", false);

  // LatentROS: pad PointCloud2 data to match real-world sensor sizes.
  // Gazebo simulated Velodyne produces ~100KB point clouds, but real sensors
  // output much larger data (VLP-16: ~700KB, VLP-32: ~1.5MB, OS1-64: ~3MB).
  // Set target_msg_size_kb to inflate the message to the desired size (0 = no padding).
  target_msg_size_kb_ = this->declare_parameter<int>("target_msg_size_kb", 0);

  // Get the input_topic_name parameter from the parameter server with default value "input"
  std::string input_topic_name = this->declare_parameter<std::string>("input_topic_name", "input");

  if (quantization_enabled_){
    //Publisher for the quantized point cloud
    pub_pointcloud_custom_ = this->create_publisher<e3_custom_messages::msg::CustomPointCloud2>(
      input_topic_name + "_custom",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
    );
  }
  else {
    // Create a point cloud publisher
    pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(input_topic_name,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  }

  // Create point cloud subscriber
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", 
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), 
    std::bind(&PointCloudInputComponent::pointCloudCb, this, std::placeholders::_1)
  );

}

size_t PointCloudInputComponent::get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg){
  //Serialize the PointCloud messages
  rclcpp::SerializedMessage serialized_data_cloud;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  const void* cloud_ptr = reinterpret_cast<const void*>(cloud_msg.get());
  cloud_serialization.serialize_message(cloud_ptr, &serialized_data_cloud);
  size_t cloud_msg_size = serialized_data_cloud.size();
  return cloud_msg_size;
}

uint32_t robotperf::perception::PointCloudInputComponent::generate_unique_key()
{
    static uint32_t counter = 1;  // Start from 1 (avoid zero confusion)
    uint32_t new_key = counter++;    
    return new_key;
}


void PointCloudInputComponent::pointCloudCb(
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{

  uint32_t unique_key = generate_unique_key();  // Generate a unique key

  // Store the key in the nanosec part of the header
  cloud_msg->header.stamp.nanosec = unique_key;

  TRACEPOINT(
    robotperf_pointcloud_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*cloud_msg)),
    cloud_msg->header.stamp.nanosec,
    cloud_msg->header.stamp.sec,
    get_msg_size(cloud_msg),
    unique_key);  // Include the unique key in tracepoints

   // -------------------------------------------
  // If quantization is DISABLED:
  // -------------------------------------------
  if (!quantization_enabled_)
  {
    // If no one is subscribed to the standard publisher, skip
    if (pub_pointcloud_->get_subscription_count() < 1) {
      return;
    }

    // LatentROS: pad point cloud to target size to match real-world sensors
    if (target_msg_size_kb_ > 0) {
      size_t target_bytes = static_cast<size_t>(target_msg_size_kb_) * 1024;
      if (cloud_msg->data.size() < target_bytes) {
        size_t original_size = cloud_msg->data.size();
        cloud_msg->data.resize(target_bytes);
        // Fill padded region by repeating original data
        for (size_t i = original_size; i < target_bytes; i++) {
          cloud_msg->data[i] = cloud_msg->data[i % original_size];
        }
        cloud_msg->row_step = target_bytes;
        cloud_msg->width = target_bytes / cloud_msg->point_step;
      }
    }

    pub_pointcloud_->publish(*cloud_msg);
  }
  else
  {
    // -------------------------------------------
    // quantization_enabled_ == true
    // -------------------------------------------
    // If no subscribers to the custom publisher, skip
    if (pub_pointcloud_custom_->get_subscription_count() < 1) {
      return;
    }
    // Convert to custom
    e3_custom_messages::msg::CustomPointCloud2 custom_msg;
    convertPointCloud2ToCustom(*cloud_msg, custom_msg);

    // Keep the unique key in the header
    custom_msg.header.stamp.sec = cloud_msg->header.stamp.sec;
    custom_msg.header.stamp.nanosec = cloud_msg->header.stamp.nanosec;

    // Publish custom
    pub_pointcloud_custom_->publish(custom_msg);
  }

  TRACEPOINT(
    robotperf_pointcloud_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*cloud_msg)),
    cloud_msg->header.stamp.nanosec,
    cloud_msg->header.stamp.sec,
    get_msg_size(cloud_msg),
    unique_key);
}

void PointCloudInputComponent::convertPointCloud2ToCustom(
  const sensor_msgs::msg::PointCloud2 & in_msg,
  e3_custom_messages::msg::CustomPointCloud2 & out_msg)
{
  using e3_custom_messages::msg::CustomPointField;

  // 1) Copy header, geometry, etc.
  out_msg.header = in_msg.header;
  out_msg.height = in_msg.height;
  out_msg.width  = in_msg.width;
  out_msg.is_bigendian = in_msg.is_bigendian; // usually false
  out_msg.is_dense = in_msg.is_dense;

  // 2) We'll store x,y,z => 3 fields, each INT16 = 2 bytes
  out_msg.fields.resize(3);

  out_msg.fields[0].name = "x";
  out_msg.fields[0].offset = 0;
  out_msg.fields[0].datatype = CustomPointField::INT16; // = 3
  out_msg.fields[0].count = 1;

  out_msg.fields[1].name = "y";
  out_msg.fields[1].offset = 2;
  out_msg.fields[1].datatype = CustomPointField::INT16;
  out_msg.fields[1].count = 1;

  out_msg.fields[2].name = "z";
  out_msg.fields[2].offset = 4;
  out_msg.fields[2].datatype = CustomPointField::INT16;
  out_msg.fields[2].count = 1;

  // 3) Each point => 6 bytes total
  out_msg.point_step = 6;
  out_msg.row_step = out_msg.point_step * out_msg.width;

  // Allocate
  size_t total_points = out_msg.width * out_msg.height;
  out_msg.data.resize(out_msg.row_step * out_msg.height);

  // We'll interpret the output data as int16 array
  int16_t* out_data = reinterpret_cast<int16_t*>(out_msg.data.data());

  // 4) Choose scale factor
  float scale = 100.0f; // e.g. 1 int16 = 0.01

  // 5) Locate x,y,z in the incoming cloud.
  //    We'll assume that the incoming cloud has x,y,z as float32
  //    at offsets 0,4,8 within each point (i.e. point_step=12).
  //    For a robust approach, parse in_msg.fields[] or handle other fields.
  
  // Let's just do a quick approach if we know the input is standard
  // x offset=0, y offset=4, z offset=8
  // Then in_msg.point_step is typically 16 or 32 etc. We'll parse carefully:

  int x_off = -1, y_off = -1, z_off = -1;
  for (auto & f : in_msg.fields)
  {
    if (f.name == "x") x_off = f.offset;
    if (f.name == "y") y_off = f.offset;
    if (f.name == "z") z_off = f.offset;
  }

  // If any are -1, the field was not found. Real code should handle that.
  if (x_off < 0 || y_off < 0 || z_off < 0)
  {
    RCLCPP_WARN(this->get_logger(), "convertPointCloud2ToCustom: x,y,z not found. Setting 0s.");
  }

  // Now read each point
  // The data is a binary blob in row-major order
  const auto * in_data = in_msg.data.data();

  for (size_t i = 0; i < total_points; i++)
  {
    // Starting address for this point
    const uint8_t* point_ptr = in_data + i * in_msg.point_step;

    float x_val = 0.0f;
    float y_val = 0.0f;
    float z_val = 0.0f;
    // Extract x,y,z if offsets are valid
    if (x_off >= 0) x_val = *reinterpret_cast<const float*>(point_ptr + x_off);
    if (y_off >= 0) y_val = *reinterpret_cast<const float*>(point_ptr + y_off);
    if (z_off >= 0) z_val = *reinterpret_cast<const float*>(point_ptr + z_off);

    // Convert to int16
    int16_t x_int = static_cast<int16_t>(std::round(x_val * scale));
    int16_t y_int = static_cast<int16_t>(std::round(y_val * scale));
    int16_t z_int = static_cast<int16_t>(std::round(z_val * scale));

    // Store in output: each point => 3 int16
    out_data[3*i + 0] = x_int;
    out_data[3*i + 1] = y_int;
    out_data[3*i + 2] = z_int;
  }
}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::PointCloudInputComponent)