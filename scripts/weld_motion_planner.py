#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from moveit_msgs.srv import GetCartesianPath
from trajectory_msgs.msg import JointTrajectory

class WeldMotionPlanner(Node):
    def __init__(self):
        super().__init__('weld_motion_planner')
        
        # 1. Subscriber: क्रैक का पूरा रास्ता (PoseArray) सुनने के लिए
        self.subscription = self.create_subscription(
            PoseArray,
            '/weld_path',
            self.path_callback,
            10
        )
        
        # 2. Publisher: रोबोट को जॉइंट कमांड (Trajectory) भेजने के लिए
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )

        # 3. MoveIt 2 Cartesian Path Service: सीधी लाइन का रास्ता बनाने के लिए
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        self.is_moving = False
        self.get_logger().info('🤖 Weld Motion Planner Ready! Waiting for Crack Path...')

    def path_callback(self, msg):
        # अगर रोबोट पहले से वेल्डिंग कर रहा है, तो नया पाथ इग्नोर करें
        if self.is_moving:
            return  

        num_waypoints = len(msg.poses)
        self.get_logger().info(f'🎯 Crack Path Received with {num_waypoints} waypoints!')
        self.plan_and_execute_cartesian(msg.poses, msg.header.frame_id)

    def plan_and_execute_cartesian(self, waypoints, frame_id):
        self.is_moving = True
        self.get_logger().info('🧠 Planning Smooth Cartesian Path for Welding...')

        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ /compute_cartesian_path service not found!')
            self.is_moving = False
            return

        # Cartesian Path रिक्वेस्ट तैयार करना
        request = GetCartesianPath.Request()
        request.header.frame_id = frame_id
        request.group_name = "ur_manipulator"  
        request.link_name = "weld_sensor_link" # यह सुनिश्चित करेगा कि सेंसर की टिप क्रैक पर चले
        request.waypoints = waypoints
        request.max_step = 0.01  # हर 1cm पर एक पॉइंट कैलकुलेट करें (High Precision)
        request.avoid_collisions = True

        future = self.cartesian_client.call_async(request)
        future.add_done_callback(self.cartesian_callback)

    def cartesian_callback(self, future):
        try:
            response = future.result()
            
            # Fraction बताता है कि MoveIt ने कितने प्रतिशत रास्ता सफलतापूर्वक बना लिया
            if response.fraction > 0.0:
                self.get_logger().info(f'✅ Path Planned Successfully! (Coverage: {response.fraction*100:.1f}%)')
                
                # MoveIt से बनी-बनाई Trajectory निकालें
                traj = response.solution.joint_trajectory
                
                # -----------------------------------------------------
                # 🕒 WELDING SPEED CONTROL (इसे धीरे चलाने के लिए)
                # -----------------------------------------------------
                current_time = 2.0  # रोबोट को पहले पॉइंट तक पहुँचने में 2 सेकंड लगेंगे
                
                for point in traj.points:
                    point.time_from_start.sec = int(current_time)
                    point.time_from_start.nanosec = int((current_time - int(current_time)) * 1e9)
                    # हर अगले पॉइंट पर जाने के लिए 0.5 सेकंड का समय लें (धीरे-धीरे वेल्डिंग)
                    current_time += 0.5 
                # -----------------------------------------------------

                # रोबोट को ट्रैजेक्टरी पब्लिश करें
                self.traj_pub.publish(traj)
                self.get_logger().info('🚀 Robot is tracing the crack smoothly (WELDING IN PROGRESS)...')
                
                # मोशन खत्म होने के बाद रोबोट को दोबारा आज़ाद करें
                total_time = current_time + 1.0
                self.timer = self.create_timer(total_time, self.reset_moving_flag)
                
            else:
                self.get_logger().warn('⚠️ MoveIt failed to plan the Cartesian Path. Path is blocked.')
                self.reset_moving_flag()
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.reset_moving_flag()

    def reset_moving_flag(self):
        self.is_moving = False
        if hasattr(self, 'timer'):
            self.timer.cancel()
        self.get_logger().info('🟢 Ready to scan the next segment of the crack...')

def main(args=None):
    rclpy.init(args=args)
    node = WeldMotionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()