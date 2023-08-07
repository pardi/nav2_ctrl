# NAV2 controller for moving a turtlebot3
This is a simple controller to move the turtlebot3 robot on a toy scenario

# Params:
- ang_max_vel -> maximum velocity for the angular motions
- lin_max_vel -> maximum velocity for the linear motions
- lookahead -> distance from the current position and the next target
- ang_tol -> tollerance for reachinng angular target
- lin_tol -> tollerance for reachinng linear target
- kp_ang -> proportional gain for the angular controller
- kp_lin -> proportional gain for the linear controller
- granularity -> granularity of the collision detection algorithm
