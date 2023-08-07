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
  plugin_name_ = name;
  
  auto node = parent.lock();
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".ang_max_vel", rclcpp::ParameterValue(M_PI / 2));  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lin_max_vel", rclcpp::ParameterValue(M_PI / 2));  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead", rclcpp::ParameterValue(0.5));    
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lin_tol", rclcpp::ParameterValue(1e-1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".ang_tol", rclcpp::ParameterValue(1e-1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_ang", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_lin", rclcpp::ParameterValue(5.0));    
       
  
  node->get_parameter(plugin_name_ + ".ang_max_vel", angular_max_velocity_);
  node->get_parameter(plugin_name_ + ".lin_max_vel", linear_max_velocity_);
  node->get_parameter(plugin_name_ + ".lookahead", lookahead_);
  node->get_parameter(plugin_name_ + ".lin_tol", linear_tolerance_);
  node->get_parameter(plugin_name_ + ".ang_tol", angular_tolerance_);
  node->get_parameter(plugin_name_ + ".kp_ang", kp_angular_);
  node->get_parameter(plugin_name_ + ".kp_lin", kp_linear_);
  node->get_parameter(plugin_name_ + ".lookahead", lookahead_);
  
}

void DexController::cleanup() {}

void DexController::activate() {}

void DexController::deactivate() {}

geometry_msgs::msg::TwistStamped DexController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*speed*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;

  // const double lookahead = 0.5;

  // auto goal = pickGoal(lookahead, pose.pose);
  // RCLCPP_INFO_STREAM(logger_, "test: " << goal.position.x << " " << lookahead_);
  RCLCPP_INFO_STREAM(logger_, "test: " << lookahead_ << " " << angular_max_velocity_ << " " << linear_max_velocity_);
  RCLCPP_INFO_STREAM(logger_, "test: " << linear_tolerance_ << " " << angular_tolerance_ << " " << kp_angular_ << " " << kp_linear_);
  
  cmd_vel.twist.angular.x = 0.1;
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
