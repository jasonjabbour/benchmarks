// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "cv_bridge/cv_bridge.h"

#include <depth_image_proc/conversions.hpp>
#include <depth_image_proc/point_cloud_xyzrgb.hpp>
#include <image_transport/image_transport.hpp>
#include "tracetools_benchmark/tracetools.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace depth_image_proc
{


PointCloudXyzrgbNode::PointCloudXyzrgbNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudXyzrgbNode", options)
{
  // Read parameters
  int queue_size = this->declare_parameter<int>("queue_size", 5);
  bool use_exact_sync = this->declare_parameter<bool>("exact_sync", false);

  // LatentROS: bypass message_filters sync for benchmarking.
  // Direct subscriptions — depth image triggers processing, rgb + info cached.
  (void)use_exact_sync;
  (void)queue_size;

  auto qos = rclcpp::QoS(10).reliable();

  // Publisher — RELIABLE
  pub_point_cloud_ = create_publisher<PointCloud2>("points", qos);

  // Cache rgb image and camera_info
  latentros_rgb_sub_ = this->create_subscription<Image>(
    "rgb/image_rect_color", qos,
    [this](Image::SharedPtr msg) { latentros_rgb_ = msg; });
  latentros_info_sub_ = this->create_subscription<CameraInfo>(
    "rgb/camera_info", qos,
    [this](CameraInfo::SharedPtr msg) {
      // Fix distortion model for image_geometry compatibility
      if (msg->distortion_model.empty() || msg->distortion_model == "pinhole") {
        msg->distortion_model = "plumb_bob";
        msg->d.resize(5, 0.0);
      }
      latentros_info_ = msg;
    });

  // Depth image triggers processing
  latentros_depth_sub_ = this->create_subscription<Image>(
    "depth_registered/image_rect", qos,
    [this](Image::SharedPtr msg) {
      if (latentros_rgb_ && latentros_info_) {
        try {
          imageCb(msg, latentros_rgb_, latentros_info_);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "LatentROS imageCb error: %s", e.what());
        }
      } else {
        RCLCPP_INFO_ONCE(get_logger(), "Waiting for RGB + camera_info...");
      }
    });
}

// Handles (un)subscribing when clients (un)subscribe
void PointCloudXyzrgbNode::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
  // if (pub_point_cloud_->getNumSubscribers() == 0)
  if (0) {
    // TODO(ros2) Implement getNumSubscribers when rcl/rmw support it
    sub_depth_.unsubscribe();
    sub_rgb_.unsubscribe();
    sub_info_.unsubscribe();
  } else if (!sub_depth_.getSubscriber()) {
    // parameter for depth_image_transport hint
    std::string depth_image_transport_param = "depth_image_transport";
    image_transport::TransportHints depth_hints(this, "raw", depth_image_transport_param);

    rclcpp::SubscriptionOptions sub_opts;
    // Update the subscription options to allow reconfigurable qos settings.
    sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions {
      {
        // Here all policies that are desired to be reconfigurable are listed.
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability,
      }};

    // depth image can use different transport.(e.g. compressedDepth)
    sub_depth_.subscribe(
      this, "depth_registered/image_rect",
      depth_hints.getTransport(), rmw_qos_profile_default, sub_opts);

    // rgb uses normal ros transport hints.
    image_transport::TransportHints hints(this, "raw");
    sub_rgb_.subscribe(
      this, "rgb/image_rect_color",
      hints.getTransport(), rmw_qos_profile_default, sub_opts);
    sub_info_.subscribe(this, "rgb/camera_info");
  }
}

void PointCloudXyzrgbNode::imageCb(
  const Image::ConstSharedPtr & depth_msg,
  const Image::ConstSharedPtr & rgb_msg_in,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  // LatentROS: kernel entry tracepoint
  TRACEPOINT(robotperf_msg_received_1,
    static_cast<const void *>(this),
    static_cast<const void *>(depth_msg.get()),
    depth_msg->header.stamp.nanosec);

  // Check for bad inputs
  if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,  // 10 seconds
      "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
  }

  // Update camera model
  model_.fromCameraInfo(info_msg);

  // Check if the input image has to be resized
  Image::ConstSharedPtr rgb_msg = rgb_msg_in;
  if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height) {
    CameraInfo info_msg_tmp = *info_msg;
    info_msg_tmp.width = depth_msg->width;
    info_msg_tmp.height = depth_msg->height;
    float ratio = static_cast<float>(depth_msg->width) / static_cast<float>(rgb_msg->width);
    info_msg_tmp.k[0] *= ratio;
    info_msg_tmp.k[2] *= ratio;
    info_msg_tmp.k[4] *= ratio;
    info_msg_tmp.k[5] *= ratio;
    info_msg_tmp.p[0] *= ratio;
    info_msg_tmp.p[2] *= ratio;
    info_msg_tmp.p[5] *= ratio;
    info_msg_tmp.p[6] *= ratio;
    model_.fromCameraInfo(info_msg_tmp);

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv_bridge::CvImage cv_rsz;
    cv_rsz.header = cv_ptr->header;
    cv_rsz.encoding = cv_ptr->encoding;
    cv::resize(
      cv_ptr->image.rowRange(0, depth_msg->height / ratio), cv_rsz.image,
      cv::Size(depth_msg->width, depth_msg->height));
    if ((rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) ||
      (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) ||
      (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8))
    {
      rgb_msg = cv_rsz.toImageMsg();
    } else {
      rgb_msg =
        cv_bridge::toCvCopy(cv_rsz.toImageMsg(), sensor_msgs::image_encodings::RGB8)->toImageMsg();
    }

    RCLCPP_ERROR(
      get_logger(), "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
      depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
    return;
  } else {
    rgb_msg = rgb_msg_in;
  }

  // Supported color encodings: RGB8, BGR8, MONO8
  int red_offset, green_offset, blue_offset, color_step;
  if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 3;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 4;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 3;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
    red_offset = 2;
    green_offset = 1;
    blue_offset = 0;
    color_step = 4;
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8) {
    red_offset = 0;
    green_offset = 0;
    blue_offset = 0;
    color_step = 1;
  } else {
    try {
      rgb_msg = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)->toImageMsg();
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
      return;
    }
    red_offset = 0;
    green_offset = 1;
    blue_offset = 2;
    color_step = 3;
  }

  auto cloud_msg = std::make_shared<PointCloud2>();
  cloud_msg->header = depth_msg->header;  // Use depth image time stamp
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  // Convert Depth Image to Pointcloud
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convertDepth<uint16_t>(depth_msg, cloud_msg, model_);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convertDepth<float>(depth_msg, cloud_msg, model_);
  } else {
    RCLCPP_ERROR(
      get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return;
  }

  // Convert RGB
  if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8) {
    convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8) {
    convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGRA8) {
    convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::RGBA8) {
    convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else if (rgb_msg->encoding == sensor_msgs::image_encodings::MONO8) {
    convertRgb(rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
  } else {
    RCLCPP_ERROR(
      get_logger(), "RGB image has unsupported encoding [%s]", rgb_msg->encoding.c_str());
    return;
  }

  // LatentROS: propagate key and kernel exit tracepoint
  cloud_msg->header.stamp.nanosec = depth_msg->header.stamp.nanosec;
  TRACEPOINT(robotperf_msg_published_1,
    static_cast<const void *>(this),
    static_cast<const void *>(cloud_msg.get()),
    depth_msg->header.stamp.nanosec);

  pub_point_cloud_->publish(*cloud_msg);
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::PointCloudXyzrgbNode)
