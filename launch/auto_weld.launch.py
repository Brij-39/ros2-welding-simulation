import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Robot Type Argument
    ur_type = LaunchConfiguration("ur_type", default="ur5e")

    # 2. Gazebo & Robot Simulation (Base control)
    sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("weld_desc"), "/launch", "/weld_sim_control.launch.py"]
        ),
        launch_arguments={"ur_type": ur_type, "launch_rviz": "false", "gui": "true"}.items(),
    )

    # 3. MoveIt & RViz (Brain & Visualization)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("weld_desc"), "/launch", "/weld_moveit_fixed.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type, 
            "use_sim_time": "true", 
            "launch_rviz": "true",
            "moveit_config_package": "weld_desc",
            "moveit_config_file": "weld.srdf.xacro",
            "description_package": "weld_desc",
            "description_file": "weld_robot.xacro"
        }.items(),
    )

    # 4. NEW: Spawn the Cracked Workpiece automatically in front of Robot
    # X=0.5 means 50cm in front, Z=0.1 means slightly above ground
    spawn_crack_block = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_crack_block",
        arguments=[
            "-entity", "crack_block",
            "-file", PathJoinSubstitution([FindPackageShare("weld_desc"), "urdf", "cracked_workpiece.urdf"]),
            "-x", "0.5", "-y", "0.0", "-z", "0.1"
        ],
        output="screen",
    )

    # We delay the block spawning by 5 seconds so Gazebo has time to load the robot first
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_crack_block])

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5e", description="Robot Type"),
        sim_control_launch,
        moveit_launch,
        delayed_spawn
    ])