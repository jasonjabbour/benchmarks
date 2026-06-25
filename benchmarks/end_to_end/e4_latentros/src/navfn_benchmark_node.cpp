/**
 * LatentROS: Simple NavFn A* benchmark wrapper.
 * No lifecycle, no action server. Pure topic-driven:
 *   PoseStamped in → A* computation → Path out
 *
 * Loads map from map_server topic, builds a static costmap,
 * and runs NavFn on every incoming goal pose.
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_navfn_planner/navfn.hpp>
using nav2_navfn_planner::NavFn;
#include "tracetools_benchmark/tracetools.h"

class NavfnBenchmarkNode : public rclcpp::Node
{
public:
  NavfnBenchmarkNode()
  : rclcpp::Node("navfn_benchmark_node")
  {
    // Subscribe to the map (one-shot, transient local)
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received map: %dx%d", msg->info.width, msg->info.height);
        map_ = msg;
        // Build costmap from the OccupancyGrid
        costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
          msg->info.width, msg->info.height,
          msg->info.resolution,
          msg->info.origin.position.x,
          msg->info.origin.position.y);
        // Copy map data into costmap
        auto * data = costmap_->getCharMap();
        for (unsigned int i = 0; i < msg->data.size(); i++) {
          if (msg->data[i] < 0) {
            data[i] = 255;  // unknown
          } else if (msg->data[i] > 50) {
            data[i] = 254;  // lethal
          } else {
            data[i] = 0;    // free
          }
        }
        // Create NavFn planner
        navfn_ = std::make_shared<NavFn>(msg->info.width, msg->info.height);
        navfn_ready_ = true;
        RCLCPP_INFO(get_logger(), "NavFn planner ready");
      });

    // Subscribe to goal poses (from rosbag or benchmark client)
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robotperf/benchmark/goal_pose",
      rclcpp::QoS(10).reliable(),
      std::bind(&NavfnBenchmarkNode::goalCallback, this, std::placeholders::_1));

    // Publish computed paths
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_plan", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "NavFn benchmark node started, waiting for map...");
  }

private:
  void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
  {
    if (!navfn_ready_) return;

    uint32_t key = goal_msg->header.stamp.nanosec;

    TRACEPOINT(robotperf_msg_received_1,
      static_cast<const void *>(this),
      static_cast<const void *>(goal_msg.get()),
      key);

    // Fixed start pose at origin
    unsigned int start_x, start_y, goal_x, goal_y;
    costmap_->worldToMap(0.0, 0.0, start_x, start_y);
    costmap_->worldToMap(
      goal_msg->pose.position.x,
      goal_msg->pose.position.y,
      goal_x, goal_y);

    // Set costmap in NavFn
    navfn_->setCostmap(costmap_->getCharMap(), true, true);

    // Set start and goal
    int start[2] = {static_cast<int>(start_x), static_cast<int>(start_y)};
    int goal[2] = {static_cast<int>(goal_x), static_cast<int>(goal_y)};
    navfn_->setStart(start);
    navfn_->setGoal(goal);

    // Compute the plan (A* search)
    navfn_->calcNavFnAstar();

    // Extract the path
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    path.header.stamp.nanosec = key;  // propagate key

    int len = navfn_->calcPath(costmap_->getSizeInCellsX() * 4);
    if (len > 0) {
      float * px = navfn_->getPathX();
      float * py = navfn_->getPathY();
      for (int i = len - 1; i >= 0; i--) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        double wx, wy;
        costmap_->mapToWorld(
          static_cast<unsigned int>(px[i]),
          static_cast<unsigned int>(py[i]),
          wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
    }

    TRACEPOINT(robotperf_msg_published_1,
      static_cast<const void *>(this),
      static_cast<const void *>(&path),
      key);

    path_pub_->publish(path);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::shared_ptr<NavFn> navfn_;
  bool navfn_ready_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavfnBenchmarkNode>());
  rclcpp::shutdown();
  return 0;
}
