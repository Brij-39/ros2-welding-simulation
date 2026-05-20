"""
weld_system.launch.py
=====================
Master launch file for the Autonomous Welding System.
Starts all required subsystems in the correct order:

  1. Gazebo physics simulation + ros2_control (robot hardware interface)
  2. MoveIt motion planning framework + RViz visualisation
  3. Cracked workpiece URDF model spawned into Gazebo (delayed by 5 s)

Dependency order matters:
  - Gazebo must be running before MoveIt connects to /joint_states
  - The workpiece must be spawned after Gazebo has finished loading the robot
    (hence the 5-second TimerAction delay)

Usage:
  ros2 launch weld_desc weld_system.launch.py
  ros2 launch weld_desc weld_system.launch.py ur_type:=ur5e
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,    # Exposes a runtime-configurable CLI argument
    IncludeLaunchDescription, # Embeds another launch file as a sub-launch
    TimerAction               # Delays one or more actions by a fixed number of seconds
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  # Locates an installed ROS2 package


def generate_launch_description():
    """
    Entry point called by 'ros2 launch'.
    Returns a LaunchDescription containing all nodes and sub-launches
    that make up the full welding system.
    """

    # ─────────────────────────────────────────────────────────────────────────
    # ARGUMENT 1 — Robot Model Selection
    # ─────────────────────────────────────────────────────────────────────────
    # LaunchConfiguration creates a runtime reference to the argument value.
    # It is resolved lazily at launch time, so the same variable can be
    # forwarded to multiple sub-launches without hardcoding the robot type.
    # Default: "ur5e" — override with ur_type:=ur3e on the command line.
    ur_type = LaunchConfiguration("ur_type", default="ur5e")

    # ─────────────────────────────────────────────────────────────────────────
    # SUB-LAUNCH 1 — Gazebo Simulation + ros2_control (Hardware Layer)
    # ─────────────────────────────────────────────────────────────────────────
    # weld_sim_control.launch.py is responsible for:
    #   - Starting Gazebo (the physics simulator)
    #   - Loading the robot URDF into the simulation
    #   - Starting ros2_control controllers (JointTrajectoryController etc.)
    #   - Publishing /joint_states and accepting /joint_trajectory commands
    #
    # launch_rviz=false → RViz is handled by the MoveIt launch below (avoids duplicates)
    # gui=true          → Show the Gazebo GUI window (set to "false" for headless CI runs)
    sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("weld_desc"), "/launch", "/weld_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type":     ur_type,
            "launch_rviz": "false",  # RViz started separately by MoveIt launch
            "gui":         "true"    # Set "false" for headless / CI environments
        }.items(),
    )

    # ─────────────────────────────────────────────────────────────────────────
    # SUB-LAUNCH 2 — MoveIt Motion Planning + RViz (Planning Layer)
    # ─────────────────────────────────────────────────────────────────────────
    # weld_moveit_fixed.launch.py is responsible for:
    #   - Starting the MoveIt move_group node (IK solver, path planning, scene monitoring)
    #   - Loading the SRDF (robot semantic description: planning groups, end-effectors)
    #   - Loading the URDF via Xacro for collision/kinematics models
    #   - Launching RViz with the MoveIt plugin for interactive planning
    #
    # use_sim_time=true         → All nodes sync to Gazebo's /clock topic (not wall clock)
    # moveit_config_package     → Package containing the MoveIt config files
    # moveit_config_file        → SRDF Xacro defining planning groups and constraints
    # description_package/file  → URDF Xacro defining the robot's links and joints
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("weld_desc"), "/launch", "/weld_moveit_fixed.launch.py"]
        ),
        launch_arguments={
            "ur_type":               ur_type,
            "use_sim_time":          "true",           # Sync to Gazebo simulation clock
            "launch_rviz":           "true",           # Open RViz with MoveIt plugin
            "moveit_config_package": "weld_desc",      # Package holding MoveIt config
            "moveit_config_file":    "weld.srdf.xacro",# Semantic robot description (SRDF)
            "description_package":   "weld_desc",      # Package holding robot URDF
            "description_file":      "weld_robot.xacro"# Full robot URDF with sensor mounts
        }.items(),
    )

    # ─────────────────────────────────────────────────────────────────────────
    # NODE — Spawn Cracked Workpiece into Gazebo
    # ─────────────────────────────────────────────────────────────────────────
    # gazebo_ros/spawn_entity.py reads a URDF file and inserts the model into
    # the running Gazebo simulation via the /spawn_entity service.
    #
    # Pose:
    #   x=0.5  → 500 mm directly in front of the robot base (within arm reach)
    #   y=0.0  → Centred on the robot's forward axis
    #   z=0.1  → 100 mm above ground — keeps the plate clear of the floor plane
    #            and avoids immediate contact/collision on spawn
    spawn_crack_block = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_crack_block",
        arguments=[
            "-entity", "crack_block",   # Unique model name inside Gazebo
            "-file", PathJoinSubstitution(
                # Resolves to: <weld_desc_install_prefix>/urdf/cracked_workpiece.urdf
                [FindPackageShare("weld_desc"), "urdf", "cracked_workpiece.urdf"]
            ),
            "-x", "0.5",   # 500 mm forward (in front of robot)
            "-y", "0.0",   # Centred laterally
            "-z", "0.1"    # 100 mm above the ground plane
        ],
        output="screen",   # Print spawn success/failure messages to the terminal
    )

    # ─────────────────────────────────────────────────────────────────────────
    # TIMER — Delay Workpiece Spawn by 5 Seconds
    # ─────────────────────────────────────────────────────────────────────────
    # Gazebo takes a few seconds to load the robot URDF, initialise physics,
    # and start the /spawn_entity service. Spawning the workpiece immediately
    # at launch time would fail because the service is not yet available.
    # A 5-second delay ensures Gazebo is fully ready before the spawn call.
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_crack_block])

    # ─────────────────────────────────────────────────────────────────────────
    # LAUNCH DESCRIPTION — Assemble All Components
    # ─────────────────────────────────────────────────────────────────────────
    # Order of items:
    #   1. DeclareLaunchArgument  → registers 'ur_type' as an overridable CLI param
    #   2. sim_control_launch     → starts immediately (Gazebo + ros2_control)
    #   3. moveit_launch          → starts immediately (MoveIt + RViz)
    #   4. delayed_spawn          → fires after 5 s (workpiece into Gazebo)
    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot variant to simulate (e.g. ur3e, ur5e, ur10e)"
        ),
        sim_control_launch,  # Layer 1: physics simulation + joint control
        moveit_launch,       # Layer 2: motion planning + visualisation
        delayed_spawn        # Layer 3: workpiece model (spawned after 5 s)
    ])