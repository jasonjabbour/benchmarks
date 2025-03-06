/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan_node.hpp"
#include "tracetools_benchmark/tracetools.h"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  // Enable or disable subscribing to quantized pointcloud message type
  quantized_enabled_ = this->declare_parameter("quantization_enabled", false);

  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  using std::placeholders::_1;

  // coditiona subscription
  if (!quantized_enabled_) {
    // The ORIGINAL logic for standard PointCloud2
    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty()) {
      tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
      tf2_->setCreateTimerInterface(timer_interface);
      tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
      message_filter_ = std::make_unique<MessageFilter>(
        sub_, *tf2_, target_frame_, input_queue_size_,
        this->get_node_logging_interface(),
        this->get_node_clock_interface());
      message_filter_->registerCallback(
        std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
    } else {  // otherwise setup direct subscription
      sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
    }
  } else {
    // For custom (quantized) message
    RCLCPP_INFO(
      this->get_logger(),
      "Quantized enabled: subscribing to CustomPointCloud2 on 'cloud_in'.");

    // We do not use message_filter_ here, so we'll handle transform in code
    // by manually converting the custom cloud to a standard PointCloud2, then calling tf2_->transform.
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);

    sub_custom_ = this->create_subscription<e3_custom_messages::msg::CustomPointCloud2>(
      "cloud_in_custom",
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloudToLaserScanNode::cloudCallbackCustom, this, _1)
    );
  }

  
  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();
  const std::chrono::milliseconds timeout(100);

  while (rclcpp::ok(context) && alive_.load())
  {
    // How many subscribers are listening to our LaserScan output?
    int subscription_count =
      pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();

    // ----------------------------------------
    // Only manage sub_ if quantization is DISABLED
    // ----------------------------------------
    if (!quantized_enabled_)
    {
      if (subscription_count > 0)
      {
        // If we have LaserScan subscribers but sub_ is not active, subscribe now
        if (!sub_.getSubscriber())
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Got a subscriber to laserscan, starting pointcloud subscriber (standard PC2)."
          );
          rclcpp::SensorDataQoS qos;
          qos.keep_last(input_queue_size_);
          sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
        }
      }
      else
      {
        // No LaserScan subscribers => no need to keep sub_ subscribed
        if (sub_.getSubscriber())
        {
          RCLCPP_INFO(
            this->get_logger(),
            "No subscribers to laserscan, shutting down pointcloud subscriber (standard PC2)."
          );
          sub_.unsubscribe();
        }
      }
    }
    // ----------------------------------------
    // If quantized_enabled_ == true, do nothing with sub_ here
    // ----------------------------------------

    // Wait briefly for graph changes
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }

  // When shutting down, if we used sub_, unsubscribe
  if (!quantized_enabled_)
  {
    sub_.unsubscribe();
  }
}


void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{

    // Extract the unique key directly from nanosec
    uint32_t unique_key = cloud_msg->header.stamp.nanosec;

    // To Calculate Network Latency
    TRACEPOINT(
        robotperf_msg_received_1,
        static_cast<const void *>(this),
        static_cast<const void *>(cloud_msg.get()),
        unique_key); // Log key in tracepoints


  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;

  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }

  // To Calculate Network Latency
  TRACEPOINT(
    robotperf_msg_published_1,
    static_cast<const void *>(this),
    static_cast<const void *>(&scan_msg),
    unique_key);   
            
  pub_->publish(std::move(scan_msg));

  // To Calculate Network Latency
  TRACEPOINT(
    robotperf_msg_published_2,
    static_cast<const void *>(this),
    static_cast<const void *>(&scan_msg),
    unique_key);
}


//-----------------------------------------------------------
// cloudCallbackCustom: for e3_custom_messages::CustomPointCloud2
//-----------------------------------------------------------
void PointCloudToLaserScanNode::cloudCallbackCustom(
  e3_custom_messages::msg::CustomPointCloud2::ConstSharedPtr cloud_msg)
{
  uint32_t unique_key = cloud_msg->header.stamp.nanosec;
  TRACEPOINT(
    robotperf_msg_received_1,
    static_cast<const void *>(this),
    static_cast<const void *>(cloud_msg.get()),
    unique_key);

  //------------------------------------------
  // 1) Convert custom cloud -> sensor_msgs::PointCloud2 (float32)
  //------------------------------------------
  // Create a new "standard" pointcloud to hold the float32 data
  auto temp_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  temp_cloud->header = cloud_msg->header;  // same timestamp, frame
  temp_cloud->height = cloud_msg->height;
  temp_cloud->width  = cloud_msg->width;
  temp_cloud->is_bigendian = cloud_msg->is_bigendian; 
  temp_cloud->is_dense = cloud_msg->is_dense;

  // We'll have 3 fields: x,y,z => each float32 (4 bytes)
  temp_cloud->fields.resize(3);
  temp_cloud->fields[0].name = "x";
  temp_cloud->fields[0].offset = 0;
  temp_cloud->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  temp_cloud->fields[0].count = 1;

  temp_cloud->fields[1].name = "y";
  temp_cloud->fields[1].offset = 4;
  temp_cloud->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  temp_cloud->fields[1].count = 1;

  temp_cloud->fields[2].name = "z";
  temp_cloud->fields[2].offset = 8;
  temp_cloud->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  temp_cloud->fields[2].count = 1;

  // So each point is 12 bytes
  temp_cloud->point_step = 12;
  temp_cloud->row_step = temp_cloud->point_step * temp_cloud->width;
  size_t total_points = temp_cloud->width * temp_cloud->height;
  temp_cloud->data.resize(temp_cloud->row_step * temp_cloud->height);

  // We'll interpret the data array as float*
  float * out_ptr = reinterpret_cast<float*>(temp_cloud->data.data());

  // The custom data is int16 for x,y,z at offsets 0,2,4 => total 6 bytes
  // We'll also assume scale=100 => 1 int16 step = 0.01 m
  float scale = 100.0f;
  const int16_t* in_data = reinterpret_cast<const int16_t*>(cloud_msg->data.data());

  for (size_t i = 0; i < total_points; i++) {
    int16_t x_int = in_data[3*i + 0];
    int16_t y_int = in_data[3*i + 1];
    int16_t z_int = in_data[3*i + 2];

    float x = x_int / scale;
    float y = y_int / scale;
    float z = z_int / scale;

    // store in out_ptr: each point => x,y,z float
    // index in out_ptr => 3*i + 0 => x, +1 => y, +2 => z
    out_ptr[3*i + 0] = x;
    out_ptr[3*i + 1] = y;
    out_ptr[3*i + 2] = z;
  }

  //------------------------------------------
  // 2) Transform the newly created float32 cloud to target_frame_
  //------------------------------------------
  if (!target_frame_.empty() && (temp_cloud->header.frame_id != target_frame_)) {
    try {
      auto transformed = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*temp_cloud, *transformed, target_frame_, tf2::durationFromSec(tolerance_));
      temp_cloud = transformed;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure (custom): " << ex.what());
      return;
    }
  }

  //------------------------------------------
  // 3) Generate LaserScan from the transformed cloud
  //------------------------------------------
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = temp_cloud->header;  // now in target_frame_ if that was set

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0f;
  scan_msg->scan_time = static_cast<float>(scan_time_);
  scan_msg->range_min = static_cast<float>(range_min_);
  scan_msg->range_max = static_cast<float>(range_max_);

  uint32_t ranges_size = static_cast<uint32_t>(
    std::ceil((scan_msg->angle_max - scan_msg->angle_min)/scan_msg->angle_increment));

  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // iterate over the float32 data
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*temp_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*temp_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*temp_cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    float x_val = *iter_x;
    float y_val = *iter_y;
    float z_val = *iter_z;

    if (std::isnan(x_val) || std::isnan(y_val) || std::isnan(z_val)) {
      continue;
    }
    if (z_val > max_height_ || z_val < min_height_) {
      continue;
    }
    double range = std::hypot(x_val, y_val);
    if (range < range_min_ || range > range_max_) {
      continue;
    }
    double angle = std::atan2(y_val, x_val);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      continue;
    }
    int index = static_cast<int>((angle - scan_msg->angle_min)/scan_msg->angle_increment);
    if ((index >= 0) && (index < static_cast<int>(ranges_size))) {
      if (range < scan_msg->ranges[index]) {
        scan_msg->ranges[index] = static_cast<float>(range);
      }
    }
  }

  TRACEPOINT(
    robotperf_msg_published_1,
    static_cast<const void *>(this),
    static_cast<const void *>(&scan_msg),
    unique_key);

  pub_->publish(std::move(scan_msg));

  TRACEPOINT(
    robotperf_msg_published_2,
    static_cast<const void *>(this),
    static_cast<const void *>(&scan_msg),
    unique_key);
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)