#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene

PLANNING_FRAME = "arm_Link"

class AddRobotScene(Node):
        
    def __init__(self):
        super().__init__('add_robot_scene_node')


        # GetPlanningScene service client
        self.get_ps_client_ = self.create_client(GetPlanningScene, "/get_planning_scene")

        while not self.get_ps_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetPlanningScene service unavailable...")

        # ApplyPlanningScene service client
        self.apply_ps_client_ = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        while not self.apply_ps_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ApplyPlanningScene service not available, waiting again...')


        self.add_box("Box0", 0.05, 0.19, 0.65, -0.03, -0.09, -0.3)
        self.add_box("Box1", 0.21, 0.15, 0.65, 0.05, -0.26, -0.3)
        self.add_box("Box2", 0.21, 0.21, 0.65, 0.05, 0.11, -0.3)
        self.add_box("Box3", 0.6, 0.55, 0.65, 0.4552, -0.06, -0.3)




    def add_box(self, id, dim_x, dim_y, dim_z, pos_x, pos_y, pos_z):

        # Get current PlanningScene
        # self.get_logger().info("Getting current PlanningScene..")
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        
        
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.header.frame_id = "arm_Link"
        co.id = id

        co_sp = SolidPrimitive()
        co_sp.type = SolidPrimitive.BOX
        co_sp.dimensions.insert(SolidPrimitive.BOX_X, dim_x)
        co_sp.dimensions.insert(SolidPrimitive.BOX_Y, dim_y)
        co_sp.dimensions.insert(SolidPrimitive.BOX_Z, dim_z)

        co_pose = Pose()
        co_pose.position.x = pos_x
        co_pose.position.y = pos_y
        co_pose.position.z = pos_z

        co.primitives.append(co_sp)
        co.primitive_poses.append(co_pose)

        # oc = ObjectColor()
        # oc.id = id
        # oc.color.a = 0.7
        # oc.color.r = 123.0
        # oc.color.g = 123.0
        # oc.color.b = 123.0

        # Put object into scene msg
        scene.world.collision_objects.append(co)
        # scene.object_colors.append(oc)


        self.get_logger().info(f"Updating PlanningScene with {id}..")


        aps_request = ApplyPlanningScene.Request()
        aps_request.scene = scene
        future_response = self.apply_ps_client_.call_async(aps_request)
        
        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info(f"Updated PlanningScene with {id}!")
        else:
            self.get_logger().error(f"Failed to update PlanningScene")


def main(args=None):

    rclpy.init(args=args)

    node = AddRobotScene()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()