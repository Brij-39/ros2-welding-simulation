"""
weld_full_system.launch.py
==========================
Master launch file for the complete Autonomous Welding Pipeline.
Starts all four subsystems in dependency order:

  1. Gazebo simulation + ros2_control  →  physics + joint control layer
  2. MoveIt + RViz                     →  motion planning + visualisation layer
  3. Scene publisher (Python)          →  inserts collision plates into MoveIt
  4. C++ motion executor (delayed 20s) →  drives the robot through the weld path

Startup timeline:
  t = 0 s   Gazebo, ros2_control, MoveIt, RViz all start in parallel
  t ~ 0 s   scene_publisher starts; it has its own internal 2 s delay before
            publishing, so MoveIt's planning scene is ready by the time it fires
  t = 20 s  move_robot_cpp starts — by this point Gazebo is fully loaded,
            MoveIt has received /joint_states, and the collision scene is set

Usage:
  ros2 launch weld_desc weld_full_system.launch.py
  ros2 launch weld_desc weld_full_system.launch.py ur_type:=ur5e
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,    # Registers a CLI-overridable launch argument
    IncludeLaunchDescription, # Embeds another launch file as a sub-launch
    TimerAction               # Delays one or more actions by a fixed number of seconds
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  # Resolves an installed ROS2 package path


def generate_launch_description():
    """
    Entry point called by 'ros2 launch'.
    Returns a LaunchDescription that wires together all nodes and sub-launches
    needed to run the full autonomous welding system.
    """

    # ─────────────────────────────────────────────────────────────────────────
    # DECLARED ARGUMENTS — CLI-Overridable Launch Parameters
    # ─────────────────────────────────────────────────────────────────────────
    # DeclareLaunchArgument makes 'ur_type' visible to 'ros2 launch --show-args'
    # and lets the user override it at runtime, e.g.: ur_type:=ur5e
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot variant to simulate (e.g. ur3e, ur5e, ur10e)"
        )
    ]

    # LaunchConfiguration creates a lazy reference to the argument value.
    # It is resolved at launch time and can be forwarded to sub-launches.
    ur_type = LaunchConfiguration("ur_type")

    # ─────────────────────────────────────────────────────────────────────────
    # KINEMATICS CONFIG PATH
    # ─────────────────────────────────────────────────────────────────────────
    # my_kinematics.yaml overrides MoveIt's default KDL solver with a custom
    # one (e.g. IKFast or TracIK) tuned for the UR5e geometry.
    # PathJoinSubstitution resolves this lazily at launch time to:
    #   <weld_desc_install_prefix>/config/ur5e/my_kinematics.yaml
    # Passing this file as a 'parameters' entry loads it into the node's
    # parameter server so MoveIt picks it up during initialisation.
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("weld_desc"), "config", "ur5e", "my_kinematics.yaml"]
    )

    # ─────────────────────────────────────────────────────────────────────────
    # SUB-LAUNCH 1 — Gazebo Simulation + ros2_control (Hardware Layer)
    # ─────────────────────────────────────────────────────────────────────────
    # weld_sim_control.launch.py handles:
    #   - Starting the Gazebo physics simulator
    #   - Spawning the UR robot URDF into the simulation
    #   - Loading ros2_control controllers (JointTrajectoryController, etc.)
    #   - Publishing /joint_states and accepting /joint_trajectory commands
    #
    # launch_rviz=false → RViz is opened by the MoveIt launch below (avoids duplicates)
    # gui=true          → Show the Gazebo GUI; set "false" for headless/CI runs
    sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("weld_desc"), "/launch", "/weld_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type":     ur_type,
            "launch_rviz": "false",  # RViz is started by the MoveIt sub-launch instead
            "gui":         "true"    # Set "false" for headless environments
        }.items(),
    )

    # ─────────────────────────────────────────────────────────────────────────
    # SUB-LAUNCH 2 — MoveIt Motion Planning + RViz (Planning Layer)
    # ─────────────────────────────────────────────────────────────────────────
    # weld_moveit_fixed.launch.py handles:
    #   - Starting move_group (IK solver, path planner, collision scene monitor)
    #   - Loading the SRDF (planning groups, end-effectors, joint limits)
    #   - Loading the URDF Xacro for link/joint geometry and collision meshes
    #   - Launching RViz with the MoveIt motion planning plugin
    #
    # use_sim_time=true         → Nodes use Gazebo's /clock instead of wall clock
    # moveit_config_package     → Package that contains the MoveIt config directory
    # moveit_config_file        → SRDF Xacro defining planning groups and constraints
    # description_package/file  → URDF Xacro with full robot geometry and sensors
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("weld_desc"), "/launch", "/weld_moveit_fixed.launch.py"]
        ),
        launch_arguments={
            "ur_type":               ur_type,
            "use_sim_time":          "true",            # Sync to Gazebo simulation clock
            "launch_rviz":           "true",            # Open RViz with MoveIt plugin
            "moveit_config_package": "weld_desc",       # Package holding MoveIt config
            "moveit_config_file":    "weld.srdf.xacro", # Semantic robot description (SRDF)
            "description_package":   "weld_desc",       # Package holding robot URDF
            "description_file":      "weld_robot.xacro" # Full robot URDF with sensor mounts
        }.items(),
    )

    # ─────────────────────────────────────────────────────────────────────────
    # NODE 1 — Scene Publisher (Collision Plates)
    # ─────────────────────────────────────────────────────────────────────────
    # scene_publisher.py adds the two steel workpiece plates as BOX
    # CollisionObjects to MoveIt's planning scene so the motion planner
    # avoids them during trajectory generation.
    # The script has its own internal 2-second startup delay, so it is safe
    # to launch it at t=0 alongside MoveIt — it will wait until move_group
    # is ready before publishing.
    scene_pub = Node(
        package="weld_desc",
        executable="scene_publisher.py",
        output="screen"   # Show collision object confirmation logs in the terminal
    )

    # ─────────────────────────────────────────────────────────────────────────
    # NODE 2 — C++ Motion Executor (Delayed 20 Seconds)
    # ─────────────────────────────────────────────────────────────────────────
    # move_robot_cpp is the main C++ node that:
    #   - Subscribes to /weld_path (published by AutoWeldMaster)
    #   - Uses MoveIt's MoveGroupInterface to plan and execute the trajectory
    #   - Drives the robot arm along the crack seam waypoints
    #
    # parameters=[robot_description_kinematics]:
    #   Loads my_kinematics.yaml into the node's parameter server.
    #   Without this, MoveIt falls back to the default KDL solver which can
    #   fail for certain UR5e poses near joint limits.
    #
    # Why 20-second delay?
    #   Gazebo + MoveIt together take ~15 seconds to fully initialise on
    #   typical hardware. The collision scene also needs to be published first.
    #   20 seconds ensures:
    #     - Gazebo is running and /joint_states is being published
    #     - MoveIt's move_group is fully up and accepting planning requests
    #     - scene_publisher has already added the collision plates
    move_robot_node = Node(
        package="weld_desc",
        executable="move_robot_cpp",
        output="screen",
        parameters=[robot_description_kinematics]  # Load custom IK solver config
    )

    # Wrap the node in a TimerAction so it starts 20 seconds after launch
    delayed_move = TimerAction(period=20.0, actions=[move_robot_node])

    # ─────────────────────────────────────────────────────────────────────────
    # LAUNCH DESCRIPTION — Assemble All Components
    # ─────────────────────────────────────────────────────────────────────────
    # declared_arguments is a list, so we concatenate with + instead of nesting.
    # Launch order:
    #   t =  0 s  → Gazebo + ros2_control  (sim_control_launch)
    #   t =  0 s  → MoveIt + RViz          (moveit_launch)
    #   t =  0 s  → Scene publisher        (scene_pub, self-delays 2 s internally)
    #   t = 20 s  → C++ motion executor    (delayed_move)
    return LaunchDescription(
        declared_arguments + [
            sim_control_launch,  # Layer 1: physics simulation + joint control
            moveit_launch,       # Layer 2: motion planning + visualisation
            scene_pub,           # Layer 3: collision scene setup
            delayed_move         # Layer 4: weld path execution (starts at t=20 s)
        ]
    )