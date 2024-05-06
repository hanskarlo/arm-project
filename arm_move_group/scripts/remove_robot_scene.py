#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene

PLANNING_FRAME = "arm_Link"

class RmRobotScene(Node):
        
    def __init__(self):
        super().__init__('remove_robot_scene_node')


        # GetPlanningScene service client
        self.get_ps_client_ = self.create_client(GetPlanningScene, "/get_planning_scene")

        while not self.get_ps_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetPlanningScene service unavailable...")
            
        # ApplyPlanningScene service client
        self.apply_ps_client_ = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        while not self.apply_ps_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ApplyPlanningScene service not available, waiting again...')


        self.remove_boxes(["Box0", "Box1", "Box2", "Box3"])



    def remove_boxes(self, idlist):

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        for id in idlist:
            co = CollisionObject()
            co.operation = CollisionObject.REMOVE
            co.header.frame_id = PLANNING_FRAME
            co.id = id

            scene.world.collision_objects.append(co)


        # self.get_logger().info(f"Updating PlanningScene with {id}")


        aps_request = ApplyPlanningScene.Request()
        aps_request.scene = scene
        future_response = self.apply_ps_client_.call_async(aps_request)
        
        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info(f"Updated PlanningScene!")
        else:
            self.get_logger().error(f"Failed to update PlanningScene")
        



def main(args=None):

    rclpy.init(args=args)

    node = RmRobotScene()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()