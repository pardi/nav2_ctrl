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
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".granularity", rclcpp::ParameterValue(100));   
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".stuck_thr", rclcpp::ParameterValue(0.01));    
       
  
  node->get_parameter(plugin_name_ + ".ang_max_vel", angular_max_velocity_);
  node->get_parameter(plugin_name_ + ".lin_max_vel", linear_max_velocity_);
  node->get_parameter(plugin_name_ + ".lookahead", lookahead_);
  node->get_parameter(plugin_name_ + ".lin_tol", linear_tolerance_);
  node->get_parameter(plugin_name_ + ".ang_tol", angular_tolerance_);
  node->get_parameter(plugin_name_ + ".kp_ang", kp_angular_);
  node->get_parameter(plugin_name_ + ".kp_lin", kp_linear_);
  node->get_parameter(plugin_name_ + ".lookahead", lookahead_);
  node->get_parameter(plugin_name_ + ".granularity", granularity_);
  node->get_parameter(plugin_name_ + ".stuck_thr", stuck_threshold_);

  costmap_ = costmap_ros;
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_->getCostmap());
  
}

void DexController::cleanup() {}

void DexController::activate() {}

void DexController::deactivate() {}

geometry_msgs::msg::TwistStamped DexController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{

  geometry_msgs::msg::TwistStamped cmd_vel;

  // Is the robot stuck?

  // Check state
  if (isStuck(velocity)){
    RCLCPP_INFO_STREAM(logger_, "Robot is stuck!");

    return cmd_vel;
  }

  auto goal = pickGoal(lookahead_, pose.pose);

  // Compute the the error between the current pose and the goal
  auto transl_error = computeTranslationError(pose.pose, goal);

  // #1 - Check if it's the final goal and if we are arrived to the end-goal
  if (isEndGoal(goal) && fabs(transl_error) < linear_tolerance_){

    // Compute errors between in orientation given the goal of the global plan    
    auto endgoal_heading_error = computeEndGoalHeadingError(pose.pose, goal);

    RCLCPP_INFO_STREAM_ONCE(logger_, "Reached End-goal position - Start rotating...");
    
    cmd_vel.twist.angular.z = PID(endgoal_heading_error, angular_tolerance_, kp_angular_, angular_max_velocity_);

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
    
  if (!isCollisionFree(pose.pose, goal, cmd_vel)){
    RCLCPP_INFO_STREAM(logger_, "Detected collision!" << transl_error);
    // Clean command
    cmd_vel.twist.angular.z = 0;
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.linear.y = 0;
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

  double dotProduct = q_curr.dot(q_goal);

  // Make sure the dot product is within the range of -1 to 1, as it may be slightly out of bounds due to numerical errors
  dotProduct = std::clamp(dotProduct, -1.0, 1.0);
  // Compute the angle between the quaternions in radians
  auto angle = 2.0 * std::acos(dotProduct);
  
  if (angle > M_PI) {
    angle = 2.0 * M_PI - angle;
  }

  RCLCPP_INFO_STREAM(logger_, "angle: " << angle);

  return angle;

}

bool DexController::isEndGoal(const geometry_msgs::msg::Pose & goal_pose){
  
  std::lock_guard<std::mutex> mlock(mutex_);
  const auto& end_goal = global_plan_.poses.back().pose;

  return (fabs(end_goal.orientation.z - goal_pose.orientation.z) < 1e-5) &&
        (fabs(end_goal.position.x - goal_pose.position.x) < 1e-5) &&
        (fabs(end_goal.position.y - goal_pose.position.y) < 1e-5);
  
}


bool DexController::isCollisionFree(const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::TwistStamped & cmd_vel){

  int num_samples = granularity_;
  double x = initial_pose.position.x;
  double y = initial_pose.position.y;
  double theta = initial_pose.orientation.z;

  // Split the trajectory in a number of samples
  double dx = (goal_pose.position.x - initial_pose.position.x) / num_samples;
  double dy = (goal_pose.position.y - initial_pose.position.y) / num_samples;;
  double dtheta = (goal_pose.orientation.z - initial_pose.orientation.z) / num_samples;

  // Check the linear trajectory if the robot thrusts ahead
  if (cmd_vel.twist.linear.x > 0){
    
    for (int i = 0; i < num_samples; ++i){
      if(!checkCollision(x + dx * i, y + dy * i, theta)){
        return false;
      }
    }
  }

  // Check the angular trajectory if the robot spins
  if (cmd_vel.twist.angular.z > 0){
    
    for (int i = 0; i < num_samples; ++i){
      if(!checkCollision(x, y, theta + dtheta * i)){
        return false;
      }
    }
  }

  return true;

}

bool DexController::checkCollision(const double x, const double y, const double theta){
    
  using namespace nav2_costmap_2d;  // NOLINT
  double footprint_cost = collision_checker_->footprintCostAtPose( x, y, theta, costmap_->getRobotFootprint());

  if (footprint_cost == static_cast<double>(NO_INFORMATION) && costmap_->getLayeredCostmap()->isTrackingUnknown())
  {
    RCLCPP_INFO_STREAM(logger_, "Possible collision ahead!");

    return false;
  }

  if (footprint_cost >= static_cast<double>(LETHAL_OBSTACLE)) {
    RCLCPP_INFO_STREAM(logger_, "Lethal collision!");

    return false;
  }

  return true;
}

bool DexController::isStuck(const geometry_msgs::msg::Twist & velocity){
  
  // Store the current twist
  if (motion_history_.size() > stuck_deque_size_){
    auto oldest_vel = motion_history_[0];
    motion_history_.pop_front();

    average_history_motion_.linear.x += (velocity.linear.x - oldest_vel.linear.x) / stuck_deque_size_;
    average_history_motion_.linear.y += (velocity.linear.y - oldest_vel.linear.y) / stuck_deque_size_;
    average_history_motion_.linear.z += (velocity.linear.z - oldest_vel.linear.z) / stuck_deque_size_;
    average_history_motion_.angular.x += (velocity.angular.x - oldest_vel.angular.z) / stuck_deque_size_;
    average_history_motion_.angular.y += (velocity.angular.y - oldest_vel.angular.y) / stuck_deque_size_;
    average_history_motion_.angular.z += (velocity.angular.z - oldest_vel.angular.z) / stuck_deque_size_;
      
    auto norm = std::sqrt( std::pow(average_history_motion_.linear.x, 2) +
                          std::pow(average_history_motion_.linear.y, 2) + 
                          std::pow(average_history_motion_.linear.z, 2) + 
                          std::pow(average_history_motion_.angular.x, 2) + 
                          std::pow(average_history_motion_.angular.y, 2) + 
                          std::pow(average_history_motion_.angular.z, 2));
    
    if (norm < stuck_threshold_){
      return true;
    }
  }else{

    if (!motion_history_.empty()){

      // Average of the array before it's full
      for (const auto& vel: motion_history_){
        average_history_motion_.linear.x += vel.linear.x;
        average_history_motion_.linear.y += vel.linear.y;
        average_history_motion_.linear.z += vel.linear.z;
        average_history_motion_.angular.x += vel.angular.x;
        average_history_motion_.angular.y += vel.angular.y;
        average_history_motion_.angular.z += vel.angular.z;
      }

      average_history_motion_.linear.x /= motion_history_.size();
      average_history_motion_.linear.y /= motion_history_.size();
      average_history_motion_.linear.z /= motion_history_.size();
      average_history_motion_.angular.x /= motion_history_.size();
      average_history_motion_.angular.y /= motion_history_.size();
      average_history_motion_.angular.z /= motion_history_.size();
    }

  }


  motion_history_.push_back(velocity);

  return false;
}


void DexController::setSpeedLimit(const double & speed_limit, const bool & percentage) {}

}  // namespace dex_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(dex_controller::DexController, nav2_core::Controller)
