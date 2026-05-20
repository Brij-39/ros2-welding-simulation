#!/usr/bin/env python3
"""
WeldMotionPlanner - ROS2 Node for Sequential Weld Path Execution via IK
Workflow:
  1. Subscribe to /weld_path (PoseArray published by AutoWeldMaster)
  2. Reverse the waypoint order for optimal approach direction
  3. For each waypoint, call MoveIt's /compute_ik service to get joint angles
  4. Publish the resulting JointTrajectory to move the robot arm
  5. Wait between points to let the robot settle before moving to the next one
  6. Skip any waypoint where IK fails and continue to the next
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class WeldMotionPlanner(Node):
    def __init__(self):
        super().__init__('weld_motion_planner')

        # --- Subscriber: Weld Path ---
        # Receives the ordered array of 6-DOF poses from AutoWeldMaster
        self.subscription = self.create_subscription(
            PoseArray,
            '/weld_path',
            self.path_callback,
            10
        )

        # --- Publisher: Joint Trajectory ---
        # Sends computed joint angle commands to the robot controller
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # --- Service Client: Inverse Kinematics ---
        # Calls MoveIt's IK service to convert a Cartesian pose into joint angles
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # --- Subscriber: Current Joint States ---
        # Needed to seed the IK solver with the robot's current configuration,
        # which helps it find a solution close to the current pose (avoids jumps)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # --- State Variables ---
        self.current_joint_state = None  # Latest joint state from /joint_states
        self.moving = False              # True while a motion command is in flight
        self.current_index = 0           # Index of the waypoint currently being executed
        self.waypoints = []              # Ordered list of Pose objects to visit
        self.frame_id = 'base_link'      # Coordinate frame of the received path
        self.timer = None                # Timer handle for delayed execution steps

        self.get_logger().info('Weld Motion Planner Ready!')

    # ─────────────────────────────────────────────────────────────────────────
    # Joint State Callback — Keep Track of Current Robot Configuration
    # ─────────────────────────────────────────────────────────────────────────
    def joint_state_callback(self, msg):
        """
        Stores the latest joint state message.
        This is passed to the IK solver as the 'seed state' so that MoveIt
        searches for an IK solution near the robot's current configuration,
        resulting in smoother and more predictable motions.
        """
        self.current_joint_state = msg

    # ─────────────────────────────────────────────────────────────────────────
    # Path Callback — Triggered Once When /weld_path is Published
    # ─────────────────────────────────────────────────────────────────────────
    def path_callback(self, msg):
        """
        Receives the full weld path as a PoseArray and begins sequential execution.
        - Ignores new paths if the robot is already executing a previous one.
        - Reverses the waypoint order so the robot approaches from the far end,
          reducing risk of the arm sweeping over already-welded material.
        """

        # Guard: reject new paths while a weld sequence is already in progress
        if self.moving:
            self.get_logger().warn('Already moving, ignoring new path!')
            return

        num_waypoints = len(msg.poses)
        if num_waypoints == 0:
            return

        self.get_logger().info(f'Crack path received ({num_waypoints} points)...')

        # Log each waypoint position for debugging / verification
        for i, pose in enumerate(msg.poses):
            self.get_logger().info(
                f"Point {i}: X={pose.position.x:.3f}, "
                f"Y={pose.position.y:.3f}, Z={pose.position.z:.3f}"
            )

        # Reverse waypoints so execution starts from the far end of the crack
        # and approaches toward the robot — avoids the arm passing over hot weld seam
        self.waypoints = list(reversed(msg.poses))

        # Use the frame from the incoming message, fall back to base_link if missing
        self.frame_id = msg.header.frame_id if msg.header.frame_id else 'base_link'
        self.current_index = 0

        self.get_logger().info('Starting point-by-point execution (Reversed order)...')

        # Block here until the IK service is available (up to 5 seconds)
        self.ik_client.wait_for_service(timeout_sec=5.0)

        # Start a 2-second timer before moving to the first waypoint
        # (gives time for any previous motion to fully settle)
        self.timer = self.create_timer(2.0, self.go_to_next_position)

    # ─────────────────────────────────────────────────────────────────────────
    # Sequential Execution — Move to Next Waypoint
    # ─────────────────────────────────────────────────────────────────────────
    def go_to_next_position(self):
        """
        Called by a timer for each waypoint. Builds an IK request for the next
        pose and dispatches it asynchronously.

        Guards against:
          - All waypoints already completed
          - Joint state not yet received (IK needs a seed state)
          - A motion already in progress (prevents overlapping commands)
        """

        # All waypoints have been executed — weld sequence complete
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All points complete! Welding Done!')
            if self.timer:
                self.timer.cancel()
            self.moving = False
            return

        # Wait if the joint state hasn't been received yet (shouldn't happen
        # after the first few messages, but safe to guard anyway)
        if self.current_joint_state is None:
            self.get_logger().warn('Joint state not yet received... waiting')
            return

        # Guard: don't issue a new command while the previous one is still executing
        if self.moving:
            return

        # Cancel the current periodic timer — a new one will be created after
        # the IK response arrives (avoids re-entrant timer firings)
        if self.timer:
            self.timer.cancel()

        pose = self.waypoints[self.current_index]

        # Override the orientation to point the tool straight down (Y-axis down).
        # Quaternion (0, 1, 0, 0) = 180° rotation around X → tool faces −Z.
        # This ensures the welding tip is always perpendicular to the surface.
        pose.orientation.x = 0.0
        pose.orientation.y = 1.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        self.get_logger().info(
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'Moving to Point {self.current_index}/{len(self.waypoints) - 1}\n'
            f'X={pose.position.x:.3f}, Y={pose.position.y:.3f}, Z={pose.position.z:.3f}\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

        # ── Build IK Request ─────────────────────────────────────────────────
        request = GetPositionIK.Request()

        # Must match the MoveIt planning group name defined in your SRDF/URDF
        request.ik_request.group_name = "welding_arm"

        # Ask MoveIt to check for self-collisions and environment collisions
        request.ik_request.avoid_collisions = True

        # Seed the IK solver with the robot's current joint configuration.
        # Starting the search from the current state leads to smoother solutions
        # and avoids large jumps or configuration flips between waypoints.
        robot_state = RobotState()
        robot_state.joint_state = self.current_joint_state
        request.ik_request.robot_state = robot_state

        # Target Cartesian pose in base_link frame
        target = PoseStamped()
        target.header.frame_id = self.frame_id
        target.pose.position.x = pose.position.x
        target.pose.position.y = pose.position.y
        target.pose.position.z = pose.position.z
        # Re-apply the tool-down orientation explicitly on the IK target as well
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 1.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        request.ik_request.pose_stamped = target

        # Mark as moving before the async call returns, so no other timer
        # invocation can issue a second IK request for the same point
        self.moving = True

        # Send IK request asynchronously; ik_callback fires when the result arrives
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    # ─────────────────────────────────────────────────────────────────────────
    # IK Response Callback — Publish Trajectory or Skip on Failure
    # ─────────────────────────────────────────────────────────────────────────
    def ik_callback(self, future):
        """
        Handles the result from MoveIt's /compute_ik service.

        On SUCCESS (error_code == 1):
          - Extracts the 6 joint angles from the IK solution
          - Publishes a JointTrajectory command with a 2-second motion time
          - Schedules the next waypoint after a 3-second settling delay

        On FAILURE:
          - Logs the error code for debugging
          - Skips the failed waypoint and moves to the next one after 1 second
            (a short delay avoids hammering the IK service with rapid retries)
        """
        response = future.result()

        if response.error_code.val == 1:  # MoveIt error code 1 = SUCCESS
            joint_angles = response.solution.joint_state.position
            self.get_logger().info(
                f'Point {self.current_index} IK Solved! '
                f'Angles: {[round(a, 3) for a in joint_angles[:6]]}'
            )

            # Build and publish the JointTrajectory command
            msg = JointTrajectory()
            msg.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint',
                'elbow_joint', 'wrist_1_joint',
                'wrist_2_joint', 'wrist_3_joint'
            ]

            point = JointTrajectoryPoint()
            point.positions = list(joint_angles[:6])  # Only first 6 joints (arm DOFs)
            point.time_from_start.sec = 2             # Execute the move within 2 seconds
            msg.points.append(point)

            self.traj_pub.publish(msg)

            self.get_logger().info(
                f'Robot moving to Point {self.current_index}... '
                f'waiting 3 seconds for motion to complete'
            )

            # Advance to the next waypoint index
            self.current_index += 1
            self.moving = False

            # Wait 3 seconds before attempting the next waypoint —
            # ensures the robot has reached the target and settled before IK is re-called
            self.timer = self.create_timer(3.0, self.go_to_next_position)

        else:
            # IK solver could not find a valid joint configuration for this pose.
            # Common causes: pose out of reach, collision constraint violated,
            # or no IK solution exists for the given orientation.
            self.get_logger().error(
                f'Point {self.current_index} IK Failed! '
                f'Error Code: {response.error_code.val} — '
                f'Skipping and moving to next point...'
            )

            # Skip the failed point and retry after a short 1-second delay
            self.current_index += 1
            self.moving = False
            self.timer = self.create_timer(1.0, self.go_to_next_position)


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Entry Point
# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = WeldMotionPlanner()
    try:
        rclpy.spin(node)  # Keep the node alive and processing callbacks
    except KeyboardInterrupt:
        pass              # Graceful shutdown on Ctrl+C
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()