import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class SeamTracker(Node):
    def __init__(self):
        super().__init__('seam_tracker')
        # Sensor ke topic ko subscribe kar rahe hain
        self.subscription = self.create_subscription(
            PointCloud2,
            '/weld_sensor/depth/points',
            self.listener_callback,
            10)
        self.get_logger().info('Autonomous Seam Tracker is Online!')

    def listener_callback(self, msg):
        # 1. Point Cloud ko NumPy array mein convert karna
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        if len(points) == 0:
            return

        # 2. Filter: Sirf wahi points rakho jo camera ke theek saamne hain
        # Hum sirf camera ke 20cm ke daire mein dekh rahe hain
        mask = (points[:, 2] < 0.5) & (np.abs(points[:, 1]) < 0.1)
        filtered_points = points[mask]

        if len(filtered_points) > 0:
            # 3. Darar dhundhna: Sabse gehra point (Lowest Z) nikalna
            target_idx = np.argmin(filtered_points[:, 2])
            target_point = filtered_points[target_idx]
            
            x, y, z = target_point
            self.get_logger().info(f'Crack Found at -> X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = SeamTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()