import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class AutoWeldTracker(Node):
    def __init__(self):
        super().__init__('auto_weld_tracker')
        
        # 1. Robot ko automatic move karne ke liye Publisher
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        # 2. Camera ka data padhne ke liye Subscriber
        self.sensor_sub = self.create_subscription(
            PointCloud2, 
            '/weld_sensor/laser_profiler/points', 
            self.sensor_callback, 
            10)

        self.get_logger().info('System Ready! Sending robot to Scanning Position...')
        
        # Thoda wait karke robot ko command bhejna
        self.timer = self.create_timer(2.0, self.go_to_scan_position)
        self.position_sent = False

    def go_to_scan_position(self):
        if self.position_sent:
            return
            
        # NEW & SAFE JOINT ANGLES
        # Ye angles robot ko dabbe se kareeb 25cm upar rakhenge
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        # Hamne shoulder_lift aur elbow ko adjust kiya hai taaki robot upar uthe
        point.positions = [-2.540, -1.819, -1.501, -1.456, 1.513, 0.232]
        point.time_from_start.sec = 4  # 4 second mein aaram se move karega
        
        msg.points.append(point)
        self.traj_pub.publish(msg)
        self.position_sent = True
        self.get_logger().info('Moving to SAFE Scanning Position... Please wait.')
        
        self.timer.cancel()

    def sensor_callback(self, msg):
        # Jab tak robot sahi position par na pahunche, tab tak ruko
        if not self.position_sent:
            return

        # 1. Sensor se raw data padhna
        raw_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # 2. ROS 2 ke structured array ko normal 2D array (N x 3) mein convert karna
        if isinstance(raw_data, np.ndarray):
            x = raw_data['x']
            y = raw_data['y']
            z = raw_data['z']
            points = np.column_stack((x, y, z))
        else:
            points = np.array(list(raw_data))

        # Agar frame mein koi data nahi hai toh aage mat badho
        if len(points) == 0:
            return

        # 3. THE FLOOR FILTER (ज़मीन को इग्नोर करना)
        # Chuki camera ab sidha niche dekh raha hai, wo dabbe ke sath zameen ko bhi dekh sakta hai.
        # Zameen dabbe se zyada gehri hogi. Isliye hum 0.3m (30cm) se door ki har cheez ignore kar denge.
        mask = points[:, 2] < 0.3
        filtered_points = points[mask]

        if len(filtered_points) > 0:
            # 4. Find the Crack (Darar dabbe par sabse gehra point hogi)
            target_idx = np.argmax(filtered_points[:, 2]) 
            x, y, z = filtered_points[target_idx]
            
            # Print Target Coordinates!
            self.get_logger().info(f'[CRACK DETECTED] -> Depth(Z): {z:.3f}m | X: {x:.3f} | Y: {y:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = AutoWeldTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()