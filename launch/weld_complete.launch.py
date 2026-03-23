import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e", description="Robot Type")
    ]
    ur_type = LaunchConfiguration("ur_type")
    
    # --- DEFINE KINEMATICS PATH ---
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("weld_desc"), "config", "ur5e", "my_kinematics.yaml"]
    )

    # 1. Gazebo & Robot (Sim Control)
    sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("weld_desc"), "/launch", "/weld_sim_control.launch.py"]
        ),
        launch_arguments={"ur_type": ur_type, "launch_rviz": "false", "gui": "true"}.items(),
    )

    # 2. MoveIt (Planning Context)
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

    # 3. Scene Objects (Plates)
    scene_pub = Node(
        package="weld_desc", executable="scene_publisher.py", output="screen"
    )

    # 4. Run C++ Motion Node (Delayed 10s)
    # FIX: Added 'parameters=[robot_description_kinematics]'
    move_robot_node = Node(
        package="weld_desc", 
        executable="move_robot_cpp", 
        output="screen",
        parameters=[robot_description_kinematics] 
    )
    delayed_move = TimerAction(period=20.0, actions=[move_robot_node])

    return LaunchDescription(declared_arguments + [sim_control_launch, moveit_launch, scene_pub, delayed_move])