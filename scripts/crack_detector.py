#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import numpy as np
import open3d as o3d

# --- TF2 Imports ---
import tf2_ros
import tf2_geometry_msgs 

# --- Math aur Smoothing Imports ---
from scipy.spatial.transform import Rotation as R_sci
import scipy.interpolate as si

class AutoWeldMaster(Node):
    def __init__(self):
        super().__init__('auto_weld_master')
        
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.sensor_sub = self.create_subscription(PointCloud2, '/weld_sensor/laser_profiler/points', self.sensor_callback, 10)
        self.path_pub = self.create_publisher(PoseArray, '/weld_path', 10)

        # TF2 Buffer aur Listener Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Master System Ready! Sending robot to Scanning Position...')
        self.timer = self.create_timer(2.0, self.go_to_scan_position)
        
        self.position_sent = False
        self.path_published = False

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
        if not self.position_sent or self.path_published:
            return

        # Base Link ka Transform Lookup
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                'base_link',             
                'weld_sensor_link',      
                rclpy.time.Time(),       
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF2 Transform abhi available nahi hai: {ex}')
            return 

        # Point Cloud Processing 
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

        if len(outlier_points) == 0: return

        numerator = np.abs(a*outlier_points[:,0] + b*outlier_points[:,1] + c*outlier_points[:,2] + d)
        denominator = np.sqrt(a**2 + b**2 + c**2)
        distances = numerator / denominator

        deep_crack_mask = distances > 0.005
        real_crack_points = outlier_points[deep_crack_mask]

        if len(real_crack_points) < 10: 
            return

        centroid = np.mean(real_crack_points, axis=0)
        dists_to_centroid = np.linalg.norm(real_crack_points - centroid, axis=1)
        start_idx = np.argmax(dists_to_centroid)
        
        current_point = real_crack_points[start_idx]
        unvisited = np.delete(real_crack_points, start_idx, axis=0)
        ordered_path = [current_point]
        
        while len(unvisited) > 0:
            dists = np.linalg.norm(unvisited - current_point, axis=1)
            nearest_idx = np.argmin(dists)
            current_point = unvisited[nearest_idx]
            ordered_path.append(current_point)
            unvisited = np.delete(unvisited, nearest_idx, axis=0)
            
        ordered_path = np.array(ordered_path)
        num_waypoints = min(15, len(ordered_path)) 

        # B-Spline Path Smoothing 
        if len(ordered_path) > 3:
            tck, u = si.splprep([ordered_path[:, 0], ordered_path[:, 1], ordered_path[:, 2]], s=0.001)
            u_new = np.linspace(0, 1, num_waypoints)
            smooth_x, smooth_y, smooth_z = si.splev(u_new, tck)
            waypoints = np.vstack((smooth_x, smooth_y, smooth_z)).T
        else:
            indices = np.linspace(0, len(ordered_path) - 1, num_waypoints, dtype=int)
            waypoints = ordered_path[indices]

        # Debug Visualization
        surface_pcd = clean_pcd.select_by_index(base_inliers)
        surface_pcd.paint_uniform_color([0.7, 0.7, 0.7]) 
        
        crack_pcd = o3d.geometry.PointCloud()
        crack_pcd.points = o3d.utility.Vector3dVector(real_crack_points)
        crack_pcd.paint_uniform_color([1.0, 0.0, 0.0]) 
        
        lines = [[i, i+1] for i in range(len(waypoints)-1)]
        colors = [[0.0, 1.0, 0.0] for _ in range(len(lines))] 
        
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(waypoints)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        
        self.get_logger().info("Opening Open3D Plot... Close the window to continue ROS node.")
        o3d.visualization.draw_geometries([surface_pcd, crack_pcd, line_set], 
                                          window_name="Crack Detection Debug",
                                          width=800, height=600)
        
        # Waypoints ko 'base_link' mein transform karna with ORIENTATION aur STANDOFF
        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link" 
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for i, wp in enumerate(waypoints):
            # 1. Travel Direction (X-Axis)
            if i < len(waypoints) - 1:
                dir_vec = waypoints[i+1] - wp
            else:
                dir_vec = wp - waypoints[i-1] if len(waypoints) > 1 else np.array([1.0, 0.0, 0.0])
                
            dir_vec = dir_vec / (np.linalg.norm(dir_vec) + 1e-6)

            # 2. Surface Normal (Z-Axis)
            z_vec = plane_normal
            z_vec = z_vec / np.linalg.norm(z_vec)

            # 3. Y-Axis = Z cross X
            y_vec = np.cross(z_vec, dir_vec)
            if np.linalg.norm(y_vec) < 1e-6:
                y_vec = np.array([0.0, 1.0, 0.0])
            y_vec = y_vec / np.linalg.norm(y_vec)

            # 4. Strictly orthogonal X-Axis = Y cross Z
            x_vec = np.cross(y_vec, z_vec)
            x_vec = x_vec / np.linalg.norm(x_vec)

            # 5. Rotation Matrix to Quaternion convert karein
            rot_mat = np.column_stack((x_vec, y_vec, z_vec))
            r = R_sci.from_matrix(rot_mat)
            quat = r.as_quat() # Format: [x, y, z, w]

            # 6. STANDOFF DISTANCE (Along Z Normal Vector)
            standoff_distance = 0.05  # 50mm Standoff
            wp_with_standoff = wp + (z_vec * standoff_distance)

            sensor_pose = PoseStamped()
            sensor_pose.pose.position.x = float(wp_with_standoff[0])
            sensor_pose.pose.position.y = float(wp_with_standoff[1])
            sensor_pose.pose.position.z = float(wp_with_standoff[2])

            sensor_pose.pose.orientation.x = float(quat[0])
            sensor_pose.pose.orientation.y = float(quat[1])
            sensor_pose.pose.orientation.z = float(quat[2])
            sensor_pose.pose.orientation.w = float(quat[3])
            
            # TF2 convert to Base frame
            base_pose = tf2_geometry_msgs.do_transform_pose(sensor_pose.pose, transform_stamped)
            pose_array.poses.append(base_pose)
        
        self.path_pub.publish(pose_array)
        self.path_published = True 
        
        self.get_logger().info(f"🔴 [SUCCESS] Published {num_waypoints} waypoints with 3D STANDOFF & ORIENTATION -> COMMAND: LASER ON")

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