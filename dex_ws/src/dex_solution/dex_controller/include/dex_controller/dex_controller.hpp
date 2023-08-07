// Copyright 2023 Dexory

#ifndef DEX_CONTROLLER__DEX_CONTROLLER_HPP_
#define DEX_CONTROLLER__DEX_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nav2_util/node_utils.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include "nav2_core/controller_exceptions.hpp"

namespace dex_controller
{

/**
 * @class dex_controller::DexController
 * @brief Dex controller plugin
 */
class DexController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for dex_controller::DexController
   */
  DexController() = default;

  /**
   * @brief Destructor for dex_controller::DexController
   */
  ~DexController() override = default;

  /**
    * @brief Configure controller on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param tf TF buffer to use
    * @param costmap_ros Costmap2DROS object of environment
    */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity,
   * with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker  Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Selects the next local goal from the global plan
   * @param lookahead Defines the distance between the selected goal and the current position
   * @param current_pose Defines the current pose of the robot
   */
  geometry_msgs::msg::Pose pickGoal(const double lookahead, const geometry_msgs::msg::Pose & current_pose);

  /**
   * @brief Computes the error in translation from the target
   * @param current_pose of the robot
   * @param goal_pose to reach
  */
  double computeTranslationError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose);
  
  /**
   * @brief Computes the error in heading from the target
   * @param current_pose of the robot
   * @param goal_pose to reach
  */
  double computeHeadingError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose);
  
  /**
   * @brief Computes the error in heading from the end_goal target
   * @param current_pose of the robot
   * @param goal_pose to reach
  */
  double computeEndGoalHeadingError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose);

  /**
   * @brief PID controller for moving the robot
   * @param error of the controller
   * @param tolerance of the error
   * @param kp proportional gain 
   * @param max_vel allowed for the controller
  */
  double PID(const double error, const double tolerance, const double kp, const double max_vel);

  /**
   * @brief Checkes whether the goal_pose is the end of the global path
   * @param goal_pose to reach
  */
  bool isEndGoal(const geometry_msgs::msg::Pose & goal_pose);

  /**
   * @brief Checkes whether the current controller will collide with the environment
   * @param initial_pose of the robot
   * @param goal_pose of the robot
   * @param cmd_vel to execute
  */
  bool isCollisionFree(const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::TwistStamped & cmd_vel);


protected:
  rclcpp::Logger logger_{rclcpp::get_logger("DexController")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;
  nav_msgs::msg::Path global_plan_;

  // External params
  std::string plugin_name_;
  double angular_max_velocity_, linear_max_velocity_;
  double linear_tolerance_, angular_tolerance_;
  double kp_angular_, kp_linear_;
  double lookahead_;
  
  std::mutex mutex_;
};

}  // namespace dex_controller

#endif  // DEX_CONTROLLER__DEX_CONTROLLER_HPP_
