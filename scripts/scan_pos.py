#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ScanPositionTest(Node):
    def __init__(self):
        super().__init__('scan_position_test')

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # ✅ Saare 30 Points (Reverse Order - Point 29 se Point 0 tak)
        self.scan_positions = [
            {'x': 0.536, 'y': 0.187, 'z': 0.215},  # Point 29
            {'x': 0.539, 'y': 0.188, 'z': 0.215},  # Point 28
            {'x': 0.541, 'y': 0.192, 'z': 0.215},  # Point 27
            {'x': 0.545, 'y': 0.196, 'z': 0.215},  # Point 26
            {'x': 0.549, 'y': 0.198, 'z': 0.215},  # Point 25
            {'x': 0.554, 'y': 0.201, 'z': 0.215},  # Point 24
            {'x': 0.558, 'y': 0.204, 'z': 0.215},  # Point 23
            {'x': 0.561, 'y': 0.207, 'z': 0.215},  # Point 22
            {'x': 0.564, 'y': 0.212, 'z': 0.215},  # Point 21
            {'x': 0.566, 'y': 0.218, 'z': 0.215},  # Point 20
            {'x': 0.569, 'y': 0.222, 'z': 0.215},  # Point 19
            {'x': 0.572, 'y': 0.225, 'z': 0.215},  # Point 18
            {'x': 0.575, 'y': 0.225, 'z': 0.215},  # Point 17
            {'x': 0.578, 'y': 0.223, 'z': 0.215},  # Point 16
            {'x': 0.582, 'y': 0.223, 'z': 0.215},  # Point 15
            {'x': 0.585, 'y': 0.223, 'z': 0.215},  # Point 14
            {'x': 0.589, 'y': 0.225, 'z': 0.215},  # Point 13
            {'x': 0.592, 'y': 0.227, 'z': 0.215},  # Point 12
            {'x': 0.597, 'y': 0.225, 'z': 0.215},  # Point 11
            {'x': 0.601, 'y': 0.221, 'z': 0.215},  # Point 10
            {'x': 0.605, 'y': 0.215, 'z': 0.215},  # Point 9
            {'x': 0.609, 'y': 0.208, 'z': 0.215},  # Point 8
            {'x': 0.613, 'y': 0.206, 'z': 0.215},  # Point 7
            {'x': 0.616, 'y': 0.208, 'z': 0.215},  # Point 6
            {'x': 0.620, 'y': 0.209, 'z': 0.215},  # Point 5
            {'x': 0.624, 'y': 0.210, 'z': 0.215},  # Point 4
            {'x': 0.627, 'y': 0.211, 'z': 0.215},  # Point 3
            {'x': 0.630, 'y': 0.211, 'z': 0.215},  # Point 2
            {'x': 0.633, 'y': 0.215, 'z': 0.215},  # Point 1
            {'x': 0.634, 'y': 0.223, 'z': 0.215},  # Point 0
        ]

        self.current_index = 0
        self.moving = False

        self.get_logger().info('Waiting for /compute_ik service...')
        self.ik_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info(
            f'Service ready! {len(self.scan_positions)} points hain — '
            f'Joint states ka wait kar raha hun...'
        )
        self.timer = self.create_timer(2.0, self.go_to_next_position)

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def go_to_next_position(self):
        # Sab points complete?
        if self.current_index >= len(self.scan_positions):
            self.get_logger().info('✅ Saare 30 points complete! Robot done!')
            self.timer.cancel()
            return

        # Joint state aaya?
        if self.current_joint_state is None:
            self.get_logger().warn('Joint state abhi nahi mila... wait kar raha hun')
            return

        # Pehli movement chal rahi hai?
        if self.moving:
            return

        self.timer.cancel()

        pos = self.scan_positions[self.current_index]
        self.get_logger().info(
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'Point {self.current_index}/{len(self.scan_positions)-1} pe ja raha hun\n'
            f'X={pos["x"]}, Y={pos["y"]}, Z={pos["z"]}\n'
            f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

        # IK Request
        request = GetPositionIK.Request()
        request.ik_request.group_name = "welding_arm"
        request.ik_request.avoid_collisions = True

        robot_state = RobotState()
        robot_state.joint_state = self.current_joint_state
        request.ik_request.robot_state = robot_state

        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = pos['x']
        target.pose.position.y = pos['y']
        target.pose.position.z = pos['z']
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 1.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 0.0

        request.ik_request.pose_stamped = target

        self.moving = True
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        response = future.result()

        if response.error_code.val == 1:  # SUCCESS
            joint_angles = response.solution.joint_state.position
            self.get_logger().info(
                f'✅ Point {self.current_index} IK Solved! '
                f'Angles: {[round(a, 3) for a in joint_angles[:6]]}'
            )

            msg = JointTrajectory()
            msg.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint',
                'elbow_joint', 'wrist_1_joint',
                'wrist_2_joint', 'wrist_3_joint'
            ]
            point = JointTrajectoryPoint()
            point.positions = list(joint_angles[:6])
            point.time_from_start.sec = 4
            msg.points.append(point)
            self.traj_pub.publish(msg)

            self.get_logger().info(
                f'🤖 Robot Point {self.current_index} pe ja raha hai... '
                f'5 sec wait kar raha hun'
            )

            self.current_index += 1
            self.moving = False
            self.timer = self.create_timer(5.0, self.go_to_next_position)

        else:
            self.get_logger().error(
                f'❌ Point {self.current_index} IK Failed! '
                f'Code: {response.error_code.val} — '
                f'Skip karke next point pe ja raha hun...'
            )
            self.current_index += 1
            self.moving = False
            self.timer = self.create_timer(2.0, self.go_to_next_position)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPositionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()