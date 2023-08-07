# NAV2 controller for moving a turtlebot3
This is a simple controller to move the turtlebot3 robot on a toy scenario

# Setup
## Enable sharing of your xserver
`xhost +local:docker`

## Open the project with vscode
The `.devcontainer` folder contains all necessary information for building and setting up the environment.

Simply,
1. Open VSCode
2. Open the folder of the project
3. A message should come up asking for `reopening the folder in the container`, say yes.
   If a message doesn't appear, on the left right of the windows, there is blue button from where we can trigger this process manually.
4. Run the script using the bring-up script
`ros2 launch dex_bringup tb3_simulation_launch.py`

# Params:
- **ang_max_vel** -> maximum velocity for the angular motions
- **lin_max_vel** -> maximum velocity for the linear motions
- **lookahead** -> distance from the current position and the next target
- **ang_tol** -> tollerance for reachinng angular target
- **lin_tol** -> tollerance for reachinng linear target
- **kp_ang** -> proportional gain for the angular controller
- **kp_lin** -> proportional gain for the linear controller
- **granularity** -> granularity of the collision detection algorithm
