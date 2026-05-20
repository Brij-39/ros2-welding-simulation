#!/usr/bin/env python3
"""
scene_publisher.py
==================
ROS2 node that registers two steel workpiece plates as MoveIt CollisionObjects
in the planning scene, so the motion planner avoids them during trajectory planning.

Context:
  - Two flat plates (left and right) are placed in front of the robot arm.
  - They are modelled as thin BOX primitives and published to /collision_object.
  - A deliberate Z offset is applied so MoveIt accepts the poses without
    immediately reporting a 'start state in collision' error (see note below).

Workflow:
  1. Node starts and waits 2 seconds for MoveIt's planning scene to initialise
  2. Publishes both CollisionObject messages once
  3. Cancels the timer — no further publishing needed
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')

        # Publish CollisionObject messages to MoveIt's planning scene topic.
        # MoveIt's PlanningSceneMonitor listens here and updates its internal
        # occupancy/collision model whenever a new object is received.
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)

        # Wait 2 seconds before publishing — gives MoveIt time to fully start
        # and subscribe to /collision_object before we send the first message.
        self.timer = self.create_timer(2.0, self.add_plates)

        # Guard flag: ensures plates are added exactly once even if the timer
        # fires multiple times before it is cancelled.
        self.plates_added = False

    # ─────────────────────────────────────────────────────────────────────────
    # add_plates — Build and Publish Both Collision Objects
    # ─────────────────────────────────────────────────────────────────────────
    def add_plates(self):
        """
        Constructs two BOX CollisionObjects representing the left and right
        workpiece plates and publishes them to MoveIt's planning scene.
        Called once by the 2-second startup timer.
        """

        # Run only once; subsequent timer callbacks are ignored until cancelled
        if self.plates_added:
            return

        self.get_logger().info('Adding collision plates to MoveIt planning scene...')

        # ── Shared Geometry: both plates have identical dimensions ────────────
        # BOX dimensions are [X length, Y width, Z height] in metres.
        # 0.5 m long × 0.25 m wide × 0.01 m thick (thin flat plate)
        plate_shape = SolidPrimitive()
        plate_shape.type = SolidPrimitive.BOX
        plate_shape.dimensions = [0.5, 0.25, 0.01]

        # ── Plate 1 — Left Workpiece ──────────────────────────────────────────
        plate1 = CollisionObject()
        plate1.header.frame_id = "world"  # Pose is expressed in the world frame
        plate1.id = "plate1"              # Unique name used to identify / remove this object later

        plate1_pose = Pose()
        plate1_pose.position.x = 0.35    # 350 mm in front of the robot base
        plate1_pose.position.y = 0.126   # 126 mm to the left of the centre line

        # --- Z OFFSET NOTE ---
        # Physical table height places the plate top surface at ~0.405 m.
        # If this value is set too high (matching the robot's current TCP height),
        # MoveIt flags the start state as 'in collision' and refuses to plan.
        # Using 0.405 m here is a tuned value that reflects real plate height
        # while keeping the start state valid in MoveIt's collision checker.
        plate1_pose.position.z = 0.405

        # No rotation — plate lies flat, aligned with the world frame axes
        plate1_pose.orientation.w = 1.0

        plate1.primitives.append(plate_shape)
        plate1.primitive_poses.append(plate1_pose)
        plate1.operation = CollisionObject.ADD  # ADD = insert into planning scene

        # ── Plate 2 — Right Workpiece ─────────────────────────────────────────
        plate2 = CollisionObject()
        plate2.header.frame_id = "world"
        plate2.id = "plate2"

        plate2_pose = Pose()
        plate2_pose.position.x = 0.35    # Same X depth as plate 1
        plate2_pose.position.y = -0.126  # 126 mm to the RIGHT (negative Y = right side)

        # Same Z offset logic as plate 1 — see note above
        plate2_pose.position.z = 0.405

        plate2_pose.orientation.w = 1.0  # No rotation — flat, axis-aligned

        # Reuse the same plate_shape geometry (both plates are identical in size)
        plate2.primitives.append(plate_shape)
        plate2.primitive_poses.append(plate2_pose)
        plate2.operation = CollisionObject.ADD

        # ── Publish Both Objects ──────────────────────────────────────────────
        # MoveIt's PlanningSceneMonitor processes these messages and adds the
        # boxes to the collision world, so trajectories will avoid them.
        self.publisher.publish(plate1)
        self.publisher.publish(plate2)

        self.get_logger().info(
            'Both collision plates added successfully to the MoveIt planning scene.'
        )

        # Mark as done and stop the timer — no further publishing is needed
        self.plates_added = True
        self.timer.cancel()


# =============================================================================
# ROS2 Entry Point
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    try:
        rclpy.spin(node)   # Keep the node alive until manually stopped
    except KeyboardInterrupt:
        pass               # Graceful shutdown on Ctrl+C
    finally:
        # Always clean up the node and ROS2 context, even if an exception occurred
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()