#include <gtest/gtest.h>

TEST(MyNodeTest, TestAddition) {

  int result = 8;

  EXPECT_EQ(result, 8);
}


//   /**
//    * @brief Selects the next local goal from the global plan
//    * @param lookahead Defines the distance between the selected goal and the current position
//    * @param current_pose Defines the current pose of the robot
//    */
//   geometry_msgs::msg::Pose pickGoal(const double lookahead, const geometry_msgs::msg::Pose & current_pose);

//   /**
//    * @brief Computes the error in translation from the target
//    * @param current_pose of the robot
//    * @param goal_pose to reach
//   */
//   double computeTranslationError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose);
  
//   /**
//    * @brief Computes the error in heading from the target
//    * @param current_pose of the robot
//    * @param goal_pose to reach
//   */
//   double computeHeadingError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose);
  
//   /**
//    * @brief Computes the error in heading from the end_goal target
//    * @param current_pose of the robot
//    * @param goal_pose to reach
//   */
//   double computeEndGoalHeadingError(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose);

//   /**
//    * @brief PID controller for moving the robot
//    * @param error of the controller
//    * @param tolerance of the error
//    * @param kp proportional gain 
//    * @param max_vel allowed for the controller
//   */
//   double PID(const double error, const double tolerance, const double kp, const double max_vel);

//   /**
//    * @brief Checkes whether the goal_pose is the end of the global path
//    * @param goal_pose to reach
//   */
//   bool isEndGoal(const geometry_msgs::msg::Pose & goal_pose);

//   /**
//    * @brief Checkes whether the current controller will collide with the environment
//    * @param initial_pose of the robot
//    * @param goal_pose of the robot
//    * @param cmd_vel to execute
//   */
//   bool isCollisionFree(const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Pose & goal_pose, const geometry_msgs::msg::TwistStamped & cmd_vel);

// private:

//   /**
//    * @brief Checks the collision on the costmap
//    * @param x position of the robot
//    * @param y position of the robot
//    * @param theta orientatino on z-axis of the robot
//   */
//   bool checkCollision(const double x, const double y, const double theta);


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
