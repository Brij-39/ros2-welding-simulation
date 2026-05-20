#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import numpy as np
import open3d as o3d
import tf2_ros
import tf2_geometry_msgs 
from scipy.spatial.transform import Rotation as R_sci
from sklearn.cluster import DBSCAN
from scipy.signal import savgol_filter

class AutoWeldMaster(Node):
    def __init__(self):
        super().__init__('auto_weld_master')
        
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.sensor_sub = self.create_subscription(PointCloud2, '/weld_sensor/laser_profiler/points', self.sensor_callback, 10)
        self.path_pub = self.create_publisher(PoseArray, '/weld_path', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Master System Ready! Sending robot to Scanning Position...')
        self.timer = self.create_timer(2.0, self.go_to_scan_position)
        
        self.position_sent = False
        self.path_published = False
        self.scan_ready = False 
        self.standoff_distance = 0.015
    def go_to_scan_position(self):
        if self.position_sent:
            return
            
        self.timer.cancel()
        
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = [-2.540, -1.819, -1.501, -1.456, 1.513, 0.232]
        point.time_from_start.sec = 4  
        msg.points.append(point)
        self.traj_pub.publish(msg)
        self.position_sent = True
        self.get_logger().info('Moving to Scan Position... Waiting 5 seconds to stabilize.')
        self.timer = self.create_timer(5.0, self.enable_scanning)

    def enable_scanning(self):
        self.scan_ready = True
        self.get_logger().info('Robot stabilized! Camera is now capturing.')
        if hasattr(self, 'timer'):
            self.timer.cancel()

    def sensor_callback(self, msg):
        if not self.scan_ready or self.path_published:
            return

        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                'base_link',             
                msg.header.frame_id,      
                rclpy.time.Time(),       
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as ex:
            return

        # Point Cloud Processing 
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = [[p[0], p[1], p[2]] for p in gen]
        if len(points_list) < 100: return 
        points = np.array(points_list, dtype=np.float32)

        # ✅ बाद में:
        mask = (points[:, 2] < 0.38) & (points[:, 2] > 0.11)
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
        angle_threshold = np.cos(np.deg2rad(30)) 
        
        ngdt_inliers = [idx for idx in base_inliers if dot_products[idx] > angle_threshold]
        outlier_cloud = clean_pcd.select_by_index(ngdt_inliers, invert=True)
        outlier_points = np.asarray(outlier_cloud.points)

        if len(outlier_points) == 0: return

        numerator = np.abs(a*outlier_points[:,0] + b*outlier_points[:,1] + c*outlier_points[:,2] + d)
        denominator = np.sqrt(a**2 + b**2 + c**2)
        distances = numerator / denominator

        deep_crack_mask = distances > 0.0025
        real_crack_points = outlier_points[deep_crack_mask]

        if len(real_crack_points) < 10: 
            return

        db = DBSCAN(eps=0.008, min_samples=5).fit(real_crack_points)
        labels = db.labels_

        if len(set(labels)) > 1:  # एक से ज्यादा clusters हैं
            # सबसे बड़ा cluster ढूंढो
            unique_labels, counts = np.unique(labels[labels >= 0], return_counts=True)
            largest_label = unique_labels[np.argmax(counts)]
            real_crack_points = real_crack_points[labels == largest_label]
            self.get_logger().info(f"DBSCAN: {len(set(labels))-1} clusters मिले — सबसे बड़ा रखा")

        if len(real_crack_points) < 10:
            return
        
        # Voxel Downsample — points कम करो पर shape रखो
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(real_crack_points)
        down_pcd = temp_pcd.voxel_down_sample(voxel_size=0.002)
        real_crack_points = np.asarray(down_pcd.points)

        # PCA — सिर्फ sorting के लिए, shape के लिए नहीं
        centroid = np.mean(real_crack_points, axis=0)
        centered = real_crack_points - centroid
        cov_matrix = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        main_axis = eigenvectors[:, np.argmax(eigenvalues)]

        # Main axis पर project करके sort करो
        projections = np.dot(real_crack_points - centroid, main_axis)
        sorted_indices = np.argsort(projections)

        # Real 3D points use करो — shape maintain होगी
        ordered_path = real_crack_points[sorted_indices]
        self.get_logger().info(f"PCA sort done ✅ — {len(ordered_path)} points")

        num_waypoints = min(40, len(ordered_path)) 

        

        indices = np.linspace(0, len(ordered_path) - 1, num_waypoints, dtype=int)
        waypoints = ordered_path[indices]

        # Savitzky-Golay smoothing — shape maintain करेगा
        window = min(7, num_waypoints if num_waypoints % 2 != 0 else num_waypoints - 1)
        waypoints[:, 0] = savgol_filter(waypoints[:, 0], window, 3)
        waypoints[:, 1] = savgol_filter(waypoints[:, 1], window, 3)

        # Surface पर project करो
        proj_z = -(a * waypoints[:, 0] + b * waypoints[:, 1] + d) / c
        waypoints[:, 2] = proj_z

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
        waypoint_pcd = o3d.geometry.PointCloud()
        waypoint_pcd.points = o3d.utility.Vector3dVector(waypoints)
        waypoint_pcd.paint_uniform_color([0.0, 1.0, 0.0])
        self.get_logger().info("Opening Open3D Plot... Close the window to continue ROS node.")
        o3d.visualization.draw_geometries([surface_pcd, crack_pcd, line_set, waypoint_pcd], 
                                  window_name="Crack Detection Debug",
                                  width=800, height=600)
        
        # 6. Orientation & Transform Setup
        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link" 
        pose_array.header.stamp = self.get_clock().now().to_msg()
        z_vec_tool = -plane_normal
        z_vec_tool = z_vec_tool / np.linalg.norm(z_vec_tool)

        # Fixed orientation — scan position की
        safe_quat = np.array([0.825, -0.564, 0.015, 0.040])

        for wp in waypoints:
            sensor_pose = PoseStamped()
            sensor_pose.header.frame_id = msg.header.frame_id
    
            sensor_pose.pose.position.x = float(wp[0])
            sensor_pose.pose.position.y = float(wp[1])
            sensor_pose.pose.position.z = float(wp[2])

            sensor_pose.pose.orientation.x = float(safe_quat[0])
            sensor_pose.pose.orientation.y = float(safe_quat[1])
            sensor_pose.pose.orientation.z = float(safe_quat[2])
            sensor_pose.pose.orientation.w = float(safe_quat[3])
    
            # Transform to base_link
            base_pose = tf2_geometry_msgs.do_transform_pose(
                sensor_pose.pose, transform_stamped)
    
            # Transform के बाद standoff add करो
            base_pose.position.z += self.standoff_distance
    
            pose_array.poses.append(base_pose)
        
        # Array ka frame strictly base_link hona chahiye
        pose_array.header.frame_id = "base_link"
        
        self.path_pub.publish(pose_array)
        self.path_published = True
        self.destroy_subscription(self.sensor_sub)
        
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