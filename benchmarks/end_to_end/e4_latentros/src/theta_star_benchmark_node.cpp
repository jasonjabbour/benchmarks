/**
 * LatentROS: Topic-driven Theta* benchmark wrapper.
 *   PoseStamped in → Theta* computation → Path out
 *
 * Uses a thin subclass of theta_star::ThetaStar to call the
 * protected initializePosn() method that allocates internal data.
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_theta_star_planner/theta_star.hpp>
#include "tracetools_benchmark/tracetools.h"

// Expose the protected initializePosn method
class ThetaStarAccessible : public theta_star::ThetaStar
{
public:
  void init() { initializePosn(size_x_ * size_y_); }
};

class ThetaStarBenchmarkNode : public rclcpp::Node
{
public:
  ThetaStarBenchmarkNode()
  : rclcpp::Node("theta_star_benchmark_node")
  {
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received map: %dx%d", msg->info.width, msg->info.height);
        costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
          msg->info.width, msg->info.height, msg->info.resolution,
          msg->info.origin.position.x, msg->info.origin.position.y);
        auto * data = costmap_->getCharMap();
        for (unsigned int i = 0; i < msg->data.size(); i++) {
          if (msg->data[i] < 0) data[i] = 255;
          else if (msg->data[i] > 50) data[i] = 254;
          else data[i] = 0;
        }
        // Configure Theta*
        theta_star_ = std::make_unique<ThetaStarAccessible>();
        theta_star_->costmap_ = costmap_.get();
        theta_star_->w_euc_cost_ = 1.0;
        theta_star_->w_traversal_cost_ = 2.0;
        theta_star_->how_many_corners_ = 8;
        theta_star_->allow_unknown_ = true;
        theta_star_->size_x_ = static_cast<int>(msg->info.width);
        theta_star_->size_y_ = static_cast<int>(msg->info.height);
        // Allocate internal node_position_ vector
        theta_star_->init();
        ready_ = true;
        RCLCPP_INFO(get_logger(), "Theta* planner ready");
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robotperf/benchmark/goal_pose", rclcpp::QoS(10).reliable(),
      std::bind(&ThetaStarBenchmarkNode::goalCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/global_plan", rclcpp::QoS(10).reliable());

    RCLCPP_INFO(get_logger(), "Theta* benchmark node started, waiting for map...");
  }

private:
  void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
  {
    if (!ready_) return;

    uint32_t key = goal_msg->header.stamp.nanosec;
    TRACEPOINT(robotperf_msg_received_1,
      static_cast<const void *>(this),
      static_cast<const void *>(goal_msg.get()), key);

    // Check if start and goal are within costmap bounds
    unsigned int mx, my;
    if (!costmap_->worldToMap(0.0, 0.0, mx, my) ||
        !costmap_->worldToMap(goal_msg->pose.position.x,
                              goal_msg->pose.position.y, mx, my)) {
      TRACEPOINT(robotperf_msg_published_1,
        static_cast<const void *>(this),
        static_cast<const void *>(goal_msg.get()), key);
      return;  // out of bounds, skip
    }

    // Set start and goal
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.position.x = 0.0;
    start.pose.position.y = 0.0;
    start.pose.orientation.w = 1.0;
    theta_star_->setStartAndGoal(start, *goal_msg);

    // Check if planning is safe
    if (theta_star_->isUnsafeToPlan()) {
      TRACEPOINT(robotperf_msg_published_1,
        static_cast<const void *>(this),
        static_cast<const void *>(goal_msg.get()), key);
      return;
    }

    // Compute path
    std::vector<coordsW> raw_path;
    theta_star_->generatePath(raw_path);

    // Build Path message
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    path.header.stamp.nanosec = key;
    for (auto & coord : raw_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = coord.x;
      pose.pose.position.y = coord.y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    TRACEPOINT(robotperf_msg_published_1,
      static_cast<const void *>(this),
      static_cast<const void *>(&path), key);
    path_pub_->publish(path);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::unique_ptr<ThetaStarAccessible> theta_star_;
  bool ready_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThetaStarBenchmarkNode>());
  rclcpp::shutdown();
  return 0;
}
