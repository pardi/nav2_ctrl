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

  auto goal = pickGoal(lookahead_, pose.pose);

  // Compute the the error between the current pose and the goal
  auto transl_error = computeTranslationError(pose.pose, goal);

  // #1 - Check if it's the final goal and if we are arrived to the end-goal
  if (isEndGoal(goal) && fabs(transl_error) < linear_tolerance_){

    // Compute errors between in orientation given the goal of the global plan    
    auto endgoal_heading_error = computeEndGoalHeadingError(pose.pose, goal);

    RCLCPP_INFO_STREAM_ONCE(logger_, "Reached End-goal position - Start rotating...");
    
    cmd_vel.twist.angular.z = PID(endgoal_heading_error, angular_tolerance_, kp_angular_, angular_max_velocity_);

    RCLCPP_INFO_STREAM(logger_, "Error " << endgoal_heading_error);
    RCLCPP_INFO_STREAM(logger_, "CMD " << cmd_vel.twist.angular.z);

  }
  else{
  
    // Compute errors between the current and goal orientation
    auto heading_error = computeHeadingError(pose.pose, goal);

    // #2 - Do I need to rotate?

    if (fabs(heading_error) >= angular_tolerance_){

      RCLCPP_DEBUG_STREAM(logger_, "Rotate with velocity " << heading_error);
      
      cmd_vel.twist.angular.z = PID(heading_error, angular_tolerance_, kp_angular_, angular_max_velocity_);

    }else{
    
      // #3 - Do I need to translate?
      RCLCPP_DEBUG_STREAM(logger_, "Translate with velocity" << transl_error);

      cmd_vel.twist.linear.x = PID(transl_error, linear_tolerance_, kp_linear_, linear_max_velocity_);
    }
  }

  cmd_vel.header.frame_id = pose.header.frame_id;
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

double DexController::computeHeadingError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose){

  // Transform rotation to quaternion and obtain current heading
  tf2::Quaternion q_curr(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
  const auto current_heading = q_curr.getAngle() * q_curr.getAxis().z();

  // Compute errors between the current and goal position
  const auto dx = goal_pose.position.x - current_pose.position.x;
  const auto dy = goal_pose.position.y - current_pose.position.y;

  // Compute angle for the desired heading
  return std::atan2(dy, dx) - current_heading;

}

double DexController::computeTranslationError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose){

  // Compute errors between the current and goal position
  const auto dx = goal_pose.position.x - current_pose.position.x;
  const auto dy = goal_pose.position.y - current_pose.position.y;

  return sqrt(pow(dx, 2) + pow(dy, 2));

}

double DexController::PID(const double error, const double tolerance, const double kp, const double max_vel){
  
  double ctrl_cmd = 0.0;
  
  if (fabs(error) > tolerance){

    // Limit the maximum velocity simmetrically
    ctrl_cmd = std::clamp(kp * error, -max_vel, max_vel);

    RCLCPP_DEBUG_STREAM(logger_, "PID cmd: " << error << " " << ctrl_cmd);
      
  }

  return ctrl_cmd;

}

double DexController::computeEndGoalHeadingError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose){

  tf2::Quaternion q_goal(goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w);
  tf2::Quaternion q_curr(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

  const auto current_heading = q_curr.getAngle() * q_curr.getAxis().z();

  // Difference between two quaternions
  auto q_diff = (q_goal * q_curr.inverse()).normalize();

  return q_diff.getAngle() * q_diff.getAxis().z() - current_heading;

}

bool DexController::isEndGoal(const geometry_msgs::msg::Pose & goal_pose){
  
  std::lock_guard<std::mutex> mlock(mutex_);
  const auto& end_goal = global_plan_.poses.back().pose;

  return (fabs(end_goal.orientation.z - goal_pose.orientation.z) < 1e-5) &&
        (fabs(end_goal.position.x - goal_pose.position.x) < 1e-5) &&
        (fabs(end_goal.position.y - goal_pose.position.y) < 1e-5);
  
}









void DexController::setSpeedLimit(const double & speed_limit, const bool & percentage) {}

}  // namespace dex_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(dex_controller::DexController, nav2_core::Controller)
