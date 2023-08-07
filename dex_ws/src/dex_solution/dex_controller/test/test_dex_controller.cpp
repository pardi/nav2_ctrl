#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <dex_controller/dex_controller.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class DexControllerCls : public dex_controller::DexController
{
public:
  DexControllerCls()
  : dex_controller::DexController()
  {

  }
  
  void setCostMap(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap){
    costmap_ = costmap;
  }

};


TEST(DexControllerTest, pickGoal) {

  auto dxc = std::make_shared<dex_controller::DexController>();

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped ps;

  ps.pose.position.x = 0;
  path.poses.push_back(ps);
  ps.pose.position.x = 1;
  path.poses.push_back(ps);
  ps.pose.position.x = 2;
  path.poses.push_back(ps);

  geometry_msgs::msg::Pose p;
  p.position.x = 0;
  p.position.y = 0;
  p.orientation.z = 0;
  
  dxc->setPlan(path);
  double lookahead = 1.1;

  auto goal = dxc->pickGoal(lookahead, p);

  EXPECT_NEAR(goal.position.x, 1.0, 1e-5);

  lookahead = 3.0;
  goal = dxc->pickGoal(lookahead, p);

  EXPECT_NEAR(goal.position.x, 2.0, 1e-5);
}

TEST(DexControllerTest, computeTranslError) {

  auto dxc = std::make_shared<dex_controller::DexController>();

  geometry_msgs::msg::Pose pose1, pose2;
  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.orientation.z = 0;
  
  pose2.position.x = 10;
  pose2.position.y = 0;
  pose2.orientation.z = 0;

  auto error = dxc->computeTranslationError(pose1, pose2);

  EXPECT_NEAR(error, 10, 1e-5);
}

TEST(DexControllerTest, computeHeadingError) {

  auto dxc = std::make_shared<dex_controller::DexController>();

  geometry_msgs::msg::Pose pose1, pose2;
  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.orientation.z = 0;
  
  pose2.position.x = 1;
  pose2.position.y = 1;
  pose2.orientation.z = M_PI / 4.0;

  auto error = dxc->computeHeadingError(pose1, pose2);

  EXPECT_NEAR(error, M_PI / 4.0, 1e-5);
}


TEST(DexControllerTest, computeEGHeadingError) {

  auto dxc = std::make_shared<dex_controller::DexController>();

  geometry_msgs::msg::Pose pose1, pose2;
  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.orientation.z = 0;
  pose1.orientation.w = 1;
  
  pose2.position.x = 0;
  pose2.position.y = 0;
  pose2.orientation.z = 0.7071068;
  pose2.orientation.w = 0.7071068 ;

  auto error = dxc->computeEndGoalHeadingError(pose1, pose2);

  EXPECT_NEAR(error, M_PI / 2.0, 1e-5);
}


TEST(DexControllerTest, pid) {

  auto dxc = std::make_shared<dex_controller::DexController>();

  double error = 1;
  double tolerance = 0.1;
  double kp = 1;
  double max_vel = 10;

  auto ctrl = dxc->PID(error, tolerance, kp, max_vel);

  EXPECT_NEAR(ctrl, 1, 1e-5);

  error = 0.01;
  ctrl = dxc->PID(error, tolerance, kp, max_vel);

  EXPECT_NEAR(ctrl, 0, 1e-5);

  error = 20;
  ctrl = dxc->PID(error, tolerance, kp, max_vel);

  EXPECT_NEAR(ctrl, 10, 1e-5);
}


TEST(DexControllerTest, isEG) {

  auto dxc = std::make_shared<dex_controller::DexController>();

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped ps;

  ps.pose.position.x = 0;
  path.poses.push_back(ps);
  ps.pose.position.x = 1;
  path.poses.push_back(ps);
  ps.pose.position.x = 2;
  path.poses.push_back(ps);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 2;
  goal_pose.position.y = 0;
  goal_pose.orientation.z = 0;

  dxc->setPlan(path);

  auto status = dxc->isEndGoal(goal_pose);

  EXPECT_TRUE(status);
}

