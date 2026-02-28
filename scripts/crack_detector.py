#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import open3d as o3d

class AutoWeldMaster(Node):
    def __init__(self):
        super().__init__('auto_weld_master')
        
        # 1. Robot ko automatic move karne ke liye Publisher (Aapka Code)
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        # 2. Camera ka data padhne ke liye Subscriber (Naya Topic)
        self.sensor_sub = self.create_subscription(
            PointCloud2, 
            '/weld_sensor/depth/points',  # Topic name checked
            self.sensor_callback, 
            10)

        self.get_logger().info('Master System Ready! Sending robot to Scanning Position...')
        
        # Thoda wait karke robot ko command bhejna
        self.timer = self.create_timer(2.0, self.go_to_scan_position)
        self.position_sent = False

    def go_to_scan_position(self):
        if self.position_sent:
            return
            
        # Robot ko workpiece ke upar laane ke angles (Aapka Code)
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [-2.540, -1.819, -1.501, -1.456, 1.513, 0.232]
        point.time_from_start.sec = 4  
        
        msg.points.append(point)
        self.traj_pub.publish(msg)
        self.position_sent = True
        self.get_logger().info('Moving to SAFE Scanning Position... Please wait.')
        
        self.timer.cancel()

    def sensor_callback(self, msg):
        # Jab tak robot sahi position par na pahunche, tab tak scan mat karo
        if not self.position_sent:
            return

        # 1. ROS 2 PointCloud ko Numpy array mein convert karna
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(gen))
        
        if len(points) < 100:
            return 

        # 2. The Floor Filter (Zameen ko ignore karna - Aapka Logic)
        mask = points[:, 2] < 0.3
        filtered_points = points[mask]

        if len(filtered_points) < 50:
            return

        # 3. Open3D format mein convert karna
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points)

        # 4. Noise Removal 
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        clean_pcd = pcd.select_by_index(ind)

        # 5. RANSAC Plane Fitting (Tedhi surface dhoondhna - Naya Brain)
        plane_model, inliers = clean_pcd.segment_plane(distance_threshold=0.002, 
                                                       ransac_n=3, 
                                                       num_iterations=1000)
        [a, b, c, d] = plane_model  

        # 6. Outliers nikalna (Jo surface par nahi hain, wahi crack hain)
        outlier_cloud = clean_pcd.select_by_index(inliers, invert=True)
        outlier_points = np.asarray(outlier_cloud.points)

        if len(outlier_points) == 0:
            self.get_logger().info("🟢 SOLID METAL BRIDGE. (No depth anomalies) -> LASER OFF")
            return

        # 7. Exact Gehrai (Relative Depth) napna
        numerator = np.abs(a*outlier_points[:,0] + b*outlier_points[:,1] + c*outlier_points[:,2] + d)
        denominator = np.sqrt(a**2 + b**2 + c**2)
        distances = numerator / denominator

        target_idx = np.argmax(distances)
        max_crack_depth = distances[target_idx]
        x, y, z = outlier_points[target_idx]

        if max_crack_depth > 0.005: 
            self.get_logger().info(f"🔴 [CRACK DETECTED] Depth: {max_crack_depth:.4f}m | X:{x:.3f} Y:{y:.3f} -> COMMAND: LASER ON")
        else:
            self.get_logger().info(f"🟢 SOLID METAL BRIDGE. Max Depth: {max_crack_depth:.4f}m -> COMMAND: LASER OFF")

def main(args=None):
    rclpy.init(args=args)
    node = AutoWeldMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()