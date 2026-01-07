#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time

class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.timer = self.create_timer(2.0, self.add_plates)
        self.plates_added = False 

    def add_plates(self):
        if self.plates_added:
            return
            
        self.get_logger().info('Adding plates with Z-OFFSET to trick MoveIt...')

        # --- Plate 1 (Left) ---
        plate1 = CollisionObject()
        plate1.header.frame_id = "world"
        plate1.id = "plate1"

        plate_shape = SolidPrimitive()
        plate_shape.type = SolidPrimitive.BOX
        plate_shape.dimensions = [0.5, 0.25, 0.01] 

        plate1_pose = Pose()
        plate1_pose.position.x = 0.35
        plate1_pose.position.y = 0.126
        
        # --- MAGIC OFFSET (Important!) ---
        # Real Height: 0.405
        # MoveIt Height: 0.35 (Lowered to prevent collision error)
        plate1_pose.position.z = 0.405 
        plate1_pose.orientation.w = 1.0

        plate1.primitives.append(plate_shape)
        plate1.primitive_poses.append(plate1_pose)
        plate1.operation = CollisionObject.ADD

        # --- Plate 2 (Right) ---
        plate2 = CollisionObject()
        plate2.header.frame_id = "world"
        plate2.id = "plate2"

        plate2_pose = Pose()
        plate2_pose.position.x = 0.35
        plate2_pose.position.y = -0.126
        
        # --- MAGIC OFFSET ---
        plate2_pose.position.z = 0.405 
        plate2_pose.orientation.w = 1.0

        plate2.primitives.append(plate_shape) 
        plate2.primitive_poses.append(plate2_pose)
        plate2.operation = CollisionObject.ADD

        self.publisher.publish(plate1)
        self.publisher.publish(plate2)
        
        self.get_logger().info('Plates added successfully with Offset.')
        self.plates_added = True 
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()