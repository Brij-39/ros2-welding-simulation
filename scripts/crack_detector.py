#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import open3d as o3d

class AutoWeldMaster(Node):
    def __init__(self):
        super().__init__('auto_weld_master')
        
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.sensor_sub = self.create_subscription(PointCloud2, '/weld_sensor/laser_profiler/points', self.sensor_callback, 10)
        self.path_pub = self.create_publisher(PoseArray, '/weld_path', 10)

        self.get_logger().info('Master System Ready! Sending robot to Scanning Position...')
        self.timer = self.create_timer(2.0, self.go_to_scan_position)
        self.position_sent = False

    def go_to_scan_position(self):
        if self.position_sent:
            return
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = [-2.540, -1.819, -1.501, -1.456, 1.513, 0.232]
        point.time_from_start.sec = 4  
        msg.points.append(point)
        self.traj_pub.publish(msg)
        self.position_sent = True
        self.get_logger().info('Moving to SAFE Scanning Position... Please wait.')
        self.timer.cancel()

    def sensor_callback(self, msg):
        if not self.position_sent:
            return

        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = [[p[0], p[1], p[2]] for p in gen]
        if len(points_list) < 100: return 
        points = np.array(points_list, dtype=np.float32)

        mask = points[:, 2] < 0.3
        filtered_points = points[mask]
        if len(filtered_points) < 50: return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        clean_pcd = pcd.select_by_index(ind)
        
        clean_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
        clean_pcd.orient_normals_towards_camera_location(np.array([0., 0., 0.]))
        
        plane_model, base_inliers = clean_pcd.segment_plane(distance_threshold=0.002, ransac_n=3, num_iterations=1000)
        [a, b, c, d] = plane_model
        
        plane_normal = np.array([a, b, c])
        plane_normal = plane_normal / np.linalg.norm(plane_normal)
        
        pcd_normals = np.asarray(clean_pcd.normals)
        dot_products = np.abs(np.dot(pcd_normals, plane_normal))
        angle_threshold = np.cos(np.deg2rad(15)) 
        
        ngdt_inliers = [idx for idx in base_inliers if dot_products[idx] > angle_threshold]
        outlier_cloud = clean_pcd.select_by_index(ngdt_inliers, invert=True)
        outlier_points = np.asarray(outlier_cloud.points)

        if len(outlier_points) == 0:
            return

        # --- THE FIX: Workpiece ke edges (kinaron) ko filter karna ---
        numerator = np.abs(a*outlier_points[:,0] + b*outlier_points[:,1] + c*outlier_points[:,2] + d)
        denominator = np.sqrt(a**2 + b**2 + c**2)
        distances = numerator / denominator

        # SIRF un points ko rakhna jo 5mm se zyada gehre hain (Asli Crack)
        deep_crack_mask = distances > 0.005
        real_crack_points = outlier_points[deep_crack_mask]

        # Agar gehre points nahi bache, iska matlab sirf edges the
        if len(real_crack_points) < 5: 
            self.get_logger().info("🟢 SOLID METAL SURFACE. -> COMMAND: LASER OFF")
            return

        # 1. Start aur End point ab SIRF ASLI CRACK POINTS me se nikalenge
        centroid = np.mean(real_crack_points, axis=0)
        dists_to_centroid = np.linalg.norm(real_crack_points - centroid, axis=1)
        p1_idx = np.argmax(dists_to_centroid)
        p1 = real_crack_points[p1_idx] # Start Point
        
        dists_to_p1 = np.linalg.norm(real_crack_points - p1, axis=1)
        p2_idx = np.argmax(dists_to_p1)
        p2 = real_crack_points[p2_idx] # End Point

        # 2. Start se End tak 5 Waypoints banana
        num_waypoints = 5
        waypoints = np.linspace(p1, p2, num_waypoints)

        pose_array = PoseArray()
        pose_array.header.frame_id = "weld_sensor_link" 
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for wp in waypoints:
            pose = Pose()
            pose.position.x = float(wp[0])
            pose.position.y = float(wp[1])
            pose.position.z = float(wp[2]) - 0.05  # Standoff Distance
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.path_pub.publish(pose_array)
        self.get_logger().info(f"🔴 [CRACK SEGMENT DETECTED] Published Path with {num_waypoints} waypoints -> COMMAND: LASER ON")

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