/**
 * LatentROS: OMPL RRT-Connect benchmark wrapper.
 *   PoseStamped in → RRT-Connect path planning → Path out
 *
 * Uses OMPL's RRTConnect planner with occupancy grid collision checking.
 * Loads map from map_server, plans on every incoming goal pose.
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>

#include "tracetools_benchmark/tracetools.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RRTBenchmarkNode : public rclcpp::Node
{
public:
  RRTBenchmarkNode()
  : rclcpp::Node("rrt_benchmark_node")
  {
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received map: %dx%d (res=%.3f, origin=%.1f,%.1f)",
          msg->info.width, msg->info.height, msg->info.resolution,
          msg->info.origin.position.x, msg->info.origin.position.y);
        map_ = msg;
        setupOMPL();
        ready_ = true;
        RCLCPP_INFO(get_logger(), "RRT-Connect planner ready");
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robotperf/benchmark/goal_pose", rclcpp::QoS(10).reliable(),
      std::bind(&RRTBenchmarkNode::goalCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/global_plan", rclcpp::QoS(10).reliable());

    RCLCPP_INFO(get_logger(), "RRT benchmark node started, waiting for map...");
  }

private:
  void setupOMPL()
  {
    double ox = map_->info.origin.position.x;
    double oy = map_->info.origin.position.y;
    double res = map_->info.resolution;
    double w = map_->info.width * res;
    double h = map_->info.height * res;

    // 2D real vector state space
    space_ = std::make_shared<ob::RealVectorStateSpace>(2);
    auto bounds = ob::RealVectorBounds(2);
    bounds.setLow(0, ox);
    bounds.setHigh(0, ox + w);
    bounds.setLow(1, oy);
    bounds.setHigh(1, oy + h);
    space_->setBounds(bounds);

    si_ = std::make_shared<ob::SpaceInformation>(space_);
    si_->setStateValidityChecker(
      [this](const ob::State *state) -> bool {
        return isStateValid(state);
      });
    si_->setStateValidityCheckingResolution(0.005);
    si_->setup();
  }

  bool isStateValid(const ob::State *state) const
  {
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    double x = s->values[0];
    double y = s->values[1];

    double ox = map_->info.origin.position.x;
    double oy = map_->info.origin.position.y;
    double res = map_->info.resolution;

    int mx = static_cast<int>((x - ox) / res);
    int my = static_cast<int>((y - oy) / res);

    if (mx < 0 || mx >= static_cast<int>(map_->info.width) ||
        my < 0 || my >= static_cast<int>(map_->info.height)) {
      return false;
    }

    int idx = my * map_->info.width + mx;
    int cost = map_->data[idx];
    // free (<50), unknown (-1 treated as passable), occupied (>=50)
    return cost >= 0 && cost < 50;
  }

  void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
  {
    if (!ready_) return;

    uint32_t key = goal_msg->header.stamp.nanosec;
    TRACEPOINT(robotperf_msg_received_1,
      static_cast<const void *>(this),
      static_cast<const void *>(goal_msg.get()), key);

    // Set start (origin) and goal
    ob::ScopedState<> start(space_);
    start[0] = 0.0;
    start[1] = 0.0;

    ob::ScopedState<> goal(space_);
    goal[0] = goal_msg->pose.position.x;
    goal[1] = goal_msg->pose.position.y;

    auto pdef = std::make_shared<ob::ProblemDefinition>(si_);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRTConnect>(si_);
    planner->setRange(0.5);  // max step length
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Solve with 1 second timeout
    ob::PlannerStatus solved = planner->solve(
      ob::timedPlannerTerminationCondition(1.0));

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    path.header.stamp.nanosec = key;

    if (solved) {
      const auto *geo_path = pdef->getSolutionPath()->as<og::PathGeometric>();
      for (unsigned int i = 0; i < geo_path->getStateCount(); ++i) {
        const auto *s = geo_path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = s->values[0];
        pose.pose.position.y = s->values[1];
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
    }

    TRACEPOINT(robotperf_msg_published_1,
      static_cast<const void *>(this),
      static_cast<const void *>(&path), key);
    path_pub_->publish(path);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::shared_ptr<ob::RealVectorStateSpace> space_;
  std::shared_ptr<ob::SpaceInformation> si_;
  bool ready_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRTBenchmarkNode>());
  rclcpp::shutdown();
  return 0;
}
