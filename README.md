ROS 2 UR5e Welding Robot Simulation

This is the M.Tech project repository for simulating a custom UR5e welding robot in ROS 2 Humble. The project involves adding a custom welding torch to the robot, setting up a Gazebo environment with a workpiece, and configuring MoveIt! 2 for motion planning.

This repository (`weld_desc`) contains all the custom package files, including:
URDF/XACRO: 3D models for the welding torch (`weld_torch.xacro`), the full robot (`weld_robot.urdf.xacro`), and the workpiece (`workpiece.xacro`).
Config: Custom MoveIt! SRDF files (`weld.srdf.xacro`) to define the new "hand" (the torch).
Launch: Custom launch files (`weld_sim_control.launch.py`, `weld_sim_moveit.launch.py`, etc.) that fix issues in the original drivers and launch the full simulation.
Source (`src`):A C++ node (`move_robot.cpp`) that uses the MoveIt! API to send motion commands to the robot.

🚀 Dependencies

Before you begin, ensure you have the following installed:
1.  ROS 2 Humble: Installed on Ubuntu 22.04.
2.  MoveIt! 2: The core packages (`sudo apt install ros-humble-moveit`).
3.  Universal Robots ROS 2 Driver:You must have the official UR drivers cloned into your `ros2_ws/src` folder. This project depends on `ur_description`, `ur_moveit_config`, etc.


🛠️ How to Build

Once the dependencies are installed and this package (`weld_desc`) is in your `ros2_ws/src` folder:

1.  Navigate to your workspace:
    cd ~/ros2_ws

2.  Build the package:
    colcon build --packages-select weld_desc
    
3.  Source the workspace:
    source install/setup.bash

▶️ How to Launch

The simulation requires two terminals.

1. Terminal 1: Launch Gazebo & Robot Controllers

This launches the Gazebo simulation, spawns the robot and the workpiece, and starts the robot controllers.

ros2 launch weld_desc weld_sim_control.launch.py ur_type:=ur5e launch_rviz:=false