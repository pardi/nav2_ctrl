// Copyright 2023 Dexory

#include "dex_controller/dex_controller.hpp"

using namespace nav2_costmap_2d;  //NOLINT

namespace dex_controller
{

void DexController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
}

void DexController::cleanup() {}

void DexController::activate() {}

void DexController::deactivate() {}

geometry_msgs::msg::TwistStamped DexController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*speed*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;

  const double lookahead = 0.5;

  auto goal = pickGoal(lookahead, pose.pose);
  
  RCLCPP_INFO_STREAM(logger_, "test: " << goal.position.x);

  
  return cmd_vel;
}

void DexController::setPlan(const nav_msgs::msg::Path & path) {
  std::lock_guard<std::mutex> mlock(mutex_);
  global_plan_ = path;
}

geometry_msgs::msg::Pose DexController::pickGoal(const double lookahead, const geometry_msgs::msg::Pose & current_pose){
  std::lock_guard<std::mutex> mlock(mutex_);

  return std::find_if(global_plan_.poses.rbegin(), global_plan_.poses.rend(), 
                      [lookahead, &current_pose](const auto & global_plan_pose){
                        
                        auto [cx, cy, cz] = current_pose.position;
                        auto [gx, gy, gz] = global_plan_pose.pose.position;

                        return sqrt(pow(cx - gx, 2) + pow(cy - gy, 2)) <= lookahead;
                      })->pose;
 
}


void DexController::setSpeedLimit(const double & speed_limit, const bool & percentage) {}

}  // namespace dex_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(dex_controller::DexController, nav2_core::Controller)
