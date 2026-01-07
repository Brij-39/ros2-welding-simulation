#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <vector>

// --- VERIFIED MATHEMATICAL CONSTANTS ---
// Pitch angle of 15 degrees (0.2618 rad) prevents UR5e wrist singularity
const double WELD_PITCH = 0.2618; 
// Z = Plate Surface (0.41) + Standoff (0.01)
const double WELD_Z = 0.57;
// X Start = Plate Min (0.35) + Margin (0.01)
const double WELD_X_START = 0.11;
// X End = Plate Max (0.85) - Margin (0.01)
const double WELD_X_END = 0.70;
// Y Center = 0.0 (Gap between plates)
const double WELD_Y = 0.0;

int main(int argc, char** argv) {
    // 1. Initialize ROS 2 Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit_cartesian_path_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("moveit_cartesian_path_node");

    // 2. Start Spinner (Required for MoveIt callbacks)
    std::thread([&node]() { rclcpp::spin(node); }).detach();

    // 3. Create MoveGroup Interface
    auto move_group = moveit::planning_interface::MoveGroupInterface(node, "ur_manipulator");
    
    // Set Planning Tolerances (Precision Mode)
    move_group.setGoalPositionTolerance(0.005);   // 1mm
    move_group.setGoalOrientationTolerance(0.05); // ~0.5 deg
    move_group.setMaxVelocityScalingFactor(0.5);  // Safety speed
    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(10);

    RCLCPP_INFO(logger, "--- STEP 1: CALCULATING APPROACH POSE ---");

    // 4. Define Orientation (Roll=180, Pitch=15, Yaw=0)
    tf2::Quaternion q;
    q.setRPY(3.14159, WELD_PITCH, 0.0);

    // 5. Define Start Pose
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header.frame_id = "world";
    start_pose.pose.orientation.x = q.x();
    start_pose.pose.orientation.y = q.y();
    start_pose.pose.orientation.z = q.z();
    start_pose.pose.orientation.w = q.w();
    start_pose.pose.position.x = WELD_X_START;
    start_pose.pose.position.y = WELD_Y;
    start_pose.pose.position.z = WELD_Z;

    // 6. Execute Approach
    move_group.setPoseTarget(start_pose);
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    
    if (move_group.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Moving to Approach Point (X:%.3f, Z:%.3f)...", WELD_X_START, WELD_Z);
        move_group.execute(approach_plan);
    } else {
        RCLCPP_ERROR(logger, "Failed to plan approach! Check Workspace/Singularity.");
        return 1;
    }

    RCLCPP_INFO(logger, "--- STEP 2: COMPUTING LINEAR WELD PATH ---");

    // 7. Define Linear Path Waypoints
    // We only push the TARGET pose. MoveIt computes the line from Current -> Target.
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose end_pose = start_pose.pose;
    end_pose.position.x = WELD_X_END; // Only X changes
    waypoints.push_back(end_pose);

    // 8. Compute Cartesian Path
    moveit_msgs::msg::RobotTrajectory trajectory;
    // Resolution: 5mm (0.005). Jump Threshold: 0.0 (Disabled for linear moves).
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    RCLCPP_INFO(logger, "Path Coverage: %.2f%%", fraction * 100.0);

    if (fraction >= 0.95) {
        RCLCPP_INFO(logger, "Path Verified. Executing Weld...");
        // Slow down for actual welding appearance
        move_group.setMaxVelocityScalingFactor(0.2); 
        move_group.execute(trajectory);
        RCLCPP_INFO(logger, "Weld Process Complete.");
    } else {
        RCLCPP_ERROR(logger, "Cartesian Path Failed! (Fraction: %.2f). Check IK Solver.", fraction);
    }

    // 9. Clean Exit
    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::shutdown();
    return 0;
}