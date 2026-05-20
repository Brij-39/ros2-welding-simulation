#!/usr/bin/env python3
"""
AutoWeldMaster - ROS2 Node for Autonomous Weld Crack Detection and Path Planning
Workflow:
  1. Move robot arm to a predefined scanning position
  2. Capture a LiDAR/laser point cloud of the target surface
  3. Detect cracks using plane segmentation + normal filtering + DBSCAN clustering
  4. Generate a smooth 40-point welding path along the crack
  5. Transform waypoints to base_link frame and publish as PoseArray
"""

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

        # --- ROS2 Publishers & Subscribers ---

        # Publishes joint trajectory commands to move the robot arm
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscribes to the 3D point cloud from the laser profiler sensor
        self.sensor_sub = self.create_subscription(
            PointCloud2,
            '/weld_sensor/laser_profiler/points',
            self.sensor_callback,
            10
        )

        # Publishes the detected weld/crack path as an array of 6-DOF poses
        self.path_pub = self.create_publisher(PoseArray, '/weld_path', 10)

        # --- TF2 Transform Setup ---
        # Used to convert sensor-frame coordinates to robot base_link frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- State Flags ---
        self.position_sent = False   # Ensures scan position command is sent only once
        self.path_published = False  # Ensures weld path is published only once
        self.scan_ready = False      # True after robot has stabilized at scan position

        # Standoff distance (meters): tool tip kept this far above the surface while welding
        self.standoff_distance = 0.015  # 15 mm

        self.get_logger().info('Master System Ready! Sending robot to Scanning Position...')

        # Timer fires after 2 seconds to send the initial scan position command
        self.timer = self.create_timer(2.0, self.go_to_scan_position)

    # ─────────────────────────────────────────────────────────────────────────
    # STEP 1 — Move Robot to Scan Position
    # ─────────────────────────────────────────────────────────────────────────
    def go_to_scan_position(self):
        """Send a joint trajectory command to move the arm to the predefined scan pose."""

        # Guard: run only once
        if self.position_sent:
            return

        self.timer.cancel()

        # Build the JointTrajectory message for a UR-style 6-DOF robot
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        # Joint angles (radians) for the scan position — arm positioned over the target surface
        point.positions = [-2.540, -1.819, -1.501, -1.456, 1.513, 0.232]
        point.time_from_start.sec = 4  # Allow 4 seconds for the motion to complete
        msg.points.append(point)

        self.traj_pub.publish(msg)
        self.position_sent = True
        self.get_logger().info('Moving to Scan Position... Waiting 5 seconds to stabilize.')

        # Wait 5 seconds for vibrations to dampen before enabling the sensor
        self.timer = self.create_timer(5.0, self.enable_scanning)

    # ─────────────────────────────────────────────────────────────────────────
    # STEP 2 — Enable Sensor Scanning After Robot Stabilises
    # ─────────────────────────────────────────────────────────────────────────
    def enable_scanning(self):
        """Called after the robot has stabilised — allow point cloud frames to be processed."""
        self.scan_ready = True
        self.get_logger().info('Robot stabilized! Camera is now capturing.')
        if hasattr(self, 'timer'):
            self.timer.cancel()

    # ─────────────────────────────────────────────────────────────────────────
    # STEP 3 — Point Cloud Processing & Path Generation (Main Callback)
    # ─────────────────────────────────────────────────────────────────────────
    def sensor_callback(self, msg):
        """
        Processes each incoming PointCloud2 frame to:
          - Filter and clean the point cloud
          - Detect the base surface plane via RANSAC
          - Isolate crack points using normal deviation + depth thresholding
          - Cluster and sort crack points into an ordered path
          - Smooth the path and publish it as a PoseArray in base_link frame
        """

        # Skip processing if robot isn't stable yet, or path is already published
        if not self.scan_ready or self.path_published:
            return

        # --- TF Lookup: sensor frame → base_link ---
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                'base_link',          # Target frame (robot base)
                msg.header.frame_id,  # Source frame (sensor/camera)
                rclpy.time.Time(),    # Latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as ex:
            # Transform not yet available — skip this frame and try again next callback
            return

        # ── 3a. Read Raw Points ──────────────────────────────────────────────
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = [[p[0], p[1], p[2]] for p in gen]

        # Need enough points for meaningful processing
        if len(points_list) < 100:
            return

        points = np.array(points_list, dtype=np.float32)

        # ── 3b. Height (Z-axis) Filter ───────────────────────────────────────
        # Keep only points within the expected height range of the workpiece.
        # This removes the robot body, table, and far-field noise.
        mask = (points[:, 2] < 0.38) & (points[:, 2] > 0.11)
        filtered_points = points[mask]

        if len(filtered_points) < 50:
            return  # Not enough surface points after filtering

        # ── 3c. Statistical Outlier Removal ─────────────────────────────────
        # Removes stray noisy points that are far from their neighbors.
        # nb_neighbors=20: each point's 20 nearest neighbors are checked
        # std_ratio=2.0: points beyond 2 standard deviations of mean distance are removed
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        clean_pcd = pcd.select_by_index(ind)

        # ── 3d. Surface Normal Estimation ────────────────────────────────────
        # Normals are used to distinguish the flat base plate from crack regions
        # (crack edges have normals pointing in a very different direction)
        clean_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30)
        )
        # Orient all normals consistently toward the camera (origin)
        clean_pcd.orient_normals_towards_camera_location(np.array([0., 0., 0.]))

        # ── 3e. RANSAC Plane Segmentation ────────────────────────────────────
        # Fits a plane (ax + by + cz + d = 0) to the dominant flat surface.
        # distance_threshold=2mm: points within 2 mm are considered inliers (on the plane)
        plane_model, base_inliers = clean_pcd.segment_plane(
            distance_threshold=0.002,
            ransac_n=3,
            num_iterations=1000
        )
        [a, b, c, d] = plane_model

        # Normalise the plane normal vector to unit length
        plane_normal = np.array([a, b, c])
        plane_normal = plane_normal / np.linalg.norm(plane_normal)

        # ── 3f. Normal-Guided Deviation Threshold (NGDT) ─────────────────────
        # Crack regions have surface normals that deviate significantly from
        # the plane normal. We keep only RANSAC inliers whose normals align
        # closely with the plane (dot product > cos(30°)) — these are the
        # true flat-surface points. The rest are crack candidates.
        pcd_normals = np.asarray(clean_pcd.normals)
        dot_products = np.abs(np.dot(pcd_normals, plane_normal))
        angle_threshold = np.cos(np.deg2rad(30))  # ~0.866

        # Keep inliers whose normals match the plane normal within 30 degrees
        ngdt_inliers = [idx for idx in base_inliers if dot_products[idx] > angle_threshold]

        # Everything NOT in that filtered set is a potential crack/defect point
        outlier_cloud = clean_pcd.select_by_index(ngdt_inliers, invert=True)
        outlier_points = np.asarray(outlier_cloud.points)

        if len(outlier_points) == 0:
            return  # No anomalies found on the surface

        # ── 3g. Depth-Based Crack Verification ───────────────────────────────
        # Compute perpendicular distance of each outlier point from the fitted plane.
        # Shallow bumps or noise will be close to the plane (< 2.5 mm).
        # Only points deeper than 2.5 mm are considered real cracks.
        numerator = np.abs(
            a * outlier_points[:, 0] +
            b * outlier_points[:, 1] +
            c * outlier_points[:, 2] + d
        )
        denominator = np.sqrt(a**2 + b**2 + c**2)
        distances = numerator / denominator

        # Threshold: > 2.5 mm below the surface plane = real crack
        deep_crack_mask = distances > 0.0025
        real_crack_points = outlier_points[deep_crack_mask]

        if len(real_crack_points) < 10:
            # Too few deep points — likely just surface noise, not a real crack
            return

        # ── 3h. DBSCAN Clustering — Keep the Largest Crack ──────────────────
        # DBSCAN groups spatially close crack points into clusters.
        # eps=8mm: two points belong to the same cluster if within 8 mm of each other
        # min_samples=5: a cluster needs at least 5 points to be valid
        db = DBSCAN(eps=0.008, min_samples=5).fit(real_crack_points)
        labels = db.labels_

        if len(set(labels)) > 1:  # More than one cluster found
            # Select only the largest cluster (most likely the main crack seam)
            unique_labels, counts = np.unique(labels[labels >= 0], return_counts=True)
            largest_label = unique_labels[np.argmax(counts)]
            real_crack_points = real_crack_points[labels == largest_label]
            self.get_logger().info(
                f"DBSCAN: {len(set(labels)) - 1} clusters found — keeping the largest"
            )

        if len(real_crack_points) < 10:
            return  # Largest cluster still too small to form a valid path

        # ── 3i. Voxel Downsampling ───────────────────────────────────────────
        # Reduces point density uniformly (2 mm voxel grid) to avoid over-dense
        # regions dominating the path, while preserving overall crack geometry.
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(real_crack_points)
        down_pcd = temp_pcd.voxel_down_sample(voxel_size=0.002)
        real_crack_points = np.asarray(down_pcd.points)

        # ── 3j. PCA — Sort Points Along the Crack's Main Axis ───────────────
        # PCA finds the direction of maximum variance (the crack's long axis).
        # We project all crack points onto this axis and sort them — this gives
        # a spatially ordered sequence from one end of the crack to the other.
        centroid = np.mean(real_crack_points, axis=0)
        centered = real_crack_points - centroid
        cov_matrix = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        main_axis = eigenvectors[:, np.argmax(eigenvalues)]  # Eigenvector with largest eigenvalue

        # Project each point onto the main axis and sort by projection value
        projections = np.dot(real_crack_points - centroid, main_axis)
        sorted_indices = np.argsort(projections)

        # Use the actual 3D positions (not the projections) to keep crack shape intact
        ordered_path = real_crack_points[sorted_indices]
        self.get_logger().info(f"PCA sort done — {len(ordered_path)} points ordered along crack axis")

        # ── 3k. Waypoint Subsampling ─────────────────────────────────────────
        # Evenly sample up to 40 waypoints from the ordered crack path.
        # Too many waypoints would slow down robot motion planning.
        num_waypoints = min(40, len(ordered_path))
        indices = np.linspace(0, len(ordered_path) - 1, num_waypoints, dtype=int)
        waypoints = ordered_path[indices]

        # ── 3l. Savitzky-Golay Smoothing ─────────────────────────────────────
        # Smooths the X and Y coordinates of waypoints without flattening the path.
        # This reduces jerky robot motion caused by small sensor noise.
        # Window must be odd and >= polynomial order (3); capped at num_waypoints.
        window = min(7, num_waypoints if num_waypoints % 2 != 0 else num_waypoints - 1)
        waypoints[:, 0] = savgol_filter(waypoints[:, 0], window, 3)  # Smooth X
        waypoints[:, 1] = savgol_filter(waypoints[:, 1], window, 3)  # Smooth Y

        # ── 3m. Project Waypoints Back onto the Detected Plane ───────────────
        # After smoothing X/Y, recalculate Z using the plane equation (ax+by+cz+d=0)
        # so that all waypoints lie exactly on the fitted surface.
        proj_z = -(a * waypoints[:, 0] + b * waypoints[:, 1] + d) / c
        waypoints[:, 2] = proj_z

        # ── 3n. Open3D Debug Visualisation ───────────────────────────────────
        # Renders an interactive 3D window showing:
        #   - Grey:  detected base surface plane
        #   - Red:   crack/defect points
        #   - Green: smoothed welding waypoints and the path line
        surface_pcd = clean_pcd.select_by_index(base_inliers)
        surface_pcd.paint_uniform_color([0.7, 0.7, 0.7])  # Grey = base surface

        crack_pcd = o3d.geometry.PointCloud()
        crack_pcd.points = o3d.utility.Vector3dVector(real_crack_points)
        crack_pcd.paint_uniform_color([1.0, 0.0, 0.0])    # Red = crack points

        # Build a line connecting consecutive waypoints to visualise the path
        lines = [[i, i + 1] for i in range(len(waypoints) - 1)]
        colors = [[0.0, 1.0, 0.0] for _ in range(len(lines))]  # Green path lines

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(waypoints)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)

        waypoint_pcd = o3d.geometry.PointCloud()
        waypoint_pcd.points = o3d.utility.Vector3dVector(waypoints)
        waypoint_pcd.paint_uniform_color([0.0, 1.0, 0.0])  # Green = waypoint dots

        self.get_logger().info("Opening Open3D Plot... Close the window to continue ROS node.")
        o3d.visualization.draw_geometries(
            [surface_pcd, crack_pcd, line_set, waypoint_pcd],
            window_name="Crack Detection Debug",
            width=800, height=600
        )

        # ─────────────────────────────────────────────────────────────────────
        # STEP 4 — Build PoseArray and Transform to base_link
        # ─────────────────────────────────────────────────────────────────────

        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        # Tool Z-axis should point into the surface (opposite of the plane normal)
        z_vec_tool = -plane_normal
        z_vec_tool = z_vec_tool / np.linalg.norm(z_vec_tool)

        # Fixed orientation quaternion (x, y, z, w) corresponding to the
        # robot's orientation at the scan position — ensures consistent approach angle
        safe_quat = np.array([0.825, -0.564, 0.015, 0.040])

        for wp in waypoints:
            # Create a PoseStamped in the sensor's coordinate frame
            sensor_pose = PoseStamped()
            sensor_pose.header.frame_id = msg.header.frame_id  # Sensor frame

            # Assign the waypoint's 3D position
            sensor_pose.pose.position.x = float(wp[0])
            sensor_pose.pose.position.y = float(wp[1])
            sensor_pose.pose.position.z = float(wp[2])

            # Assign the fixed tool orientation
            sensor_pose.pose.orientation.x = float(safe_quat[0])
            sensor_pose.pose.orientation.y = float(safe_quat[1])
            sensor_pose.pose.orientation.z = float(safe_quat[2])
            sensor_pose.pose.orientation.w = float(safe_quat[3])

            # Transform the pose from sensor frame into robot base_link frame
            base_pose = tf2_geometry_msgs.do_transform_pose(
                sensor_pose.pose, transform_stamped
            )

            # Add standoff distance along Z after transform so the tool
            # hovers above the surface rather than touching it directly
            base_pose.position.z += self.standoff_distance

            pose_array.poses.append(base_pose)

        # Confirm the output frame is base_link before publishing
        pose_array.header.frame_id = "base_link"

        # Publish the complete weld path
        self.path_pub.publish(pose_array)
        self.path_published = True

        # Unsubscribe from the sensor — path is computed, no more frames needed
        self.destroy_subscription(self.sensor_sub)

        self.get_logger().info(
            f"[SUCCESS] Published {num_waypoints} waypoints with "
            f"3D STANDOFF & ORIENTATION -> COMMAND: LASER ON"
        )


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Entry Point
# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = AutoWeldMaster()
    try:
        rclpy.spin(node)  # Keep the node alive and processing callbacks
    except KeyboardInterrupt:
        pass  # Graceful shutdown on Ctrl+C
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()