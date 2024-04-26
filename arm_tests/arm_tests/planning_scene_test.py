import pickle

import rclpy
from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene


class CollisionObjectTest(Node):
        
    def __init__(self):
        super().__init__('move_to_saved_test_node')


        # GetPlanningScene service client
        self.get_ps_client_ = self.create_client(GetPlanningScene, "/get_planning_scene")

        while not self.get_ps_client_.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("GetPlanningScene service unavailable...")


        # ApplyPlanningScene service client
        self.apply_ps_client_ = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        while not self.apply_ps_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ApplyPlanningScene service not available, waiting again...')





    def add_table(self):

        # Get current PlanningScene
        self.get_logger().info("Getting current PlanningScene..")
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        
        
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.header.frame_id = "base_link"
        co.id = "table"

        co_sp = SolidPrimitive()
        co_sp.type = SolidPrimitive.BOX
        co_sp.dimensions.insert(SolidPrimitive.BOX_X, 0.91)
        co_sp.dimensions.insert(SolidPrimitive.BOX_Y, 1.54)
        co_sp.dimensions.insert(SolidPrimitive.BOX_Z, 0.08)

        co_pose = Pose()
        co_pose.position.x = 0.405
        co_pose.position.y = -0.67
        co_pose.position.z = -0.07

        co.primitives.append(co_sp)
        co.primitive_poses.append(co_pose)

        # Put object into scene msg
        scene.world.collision_objects.append(co)


        self.get_logger().info(f"Updating PlanningScene with new object..")


        aps_request = ApplyPlanningScene.Request()
        aps_request.scene = scene
        future_response = self.apply_ps_client_.call_async(aps_request)
        
        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info(f"Updated PlanningScene with new object!")
        else:
            self.get_logger().error(f"Failed to update PlanningScene")




    def remove_table(self):

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.header.frame_id = "base_link"
        co.id = "table"

        scene.world.collision_objects.append(co)


        self.get_logger().info(f"Updating PlanningScene with new object..")


        aps_request = ApplyPlanningScene.Request()
        aps_request.scene = scene
        future_response = self.apply_ps_client_.call_async(aps_request)
        
        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info(f"Updated PlanningScene with new object!")
        else:
            self.get_logger().error(f"Failed to update PlanningScene")
        



    def save_test(self):

        scene_name = input("Choose name for planning scene: ")


        # Get current PlanningScene
        blank_rq = GetPlanningScene.Request() # Service responds with full information on blank request
        future_response = self.get_ps_client_.call_async(blank_rq)

        rclpy.spin_until_future_complete(self, future_response)

        scene = future_response.result().scene
        scene.name = scene_name

        file_name = scene_name + ".scene"
        file_path = "/home/arodev0/arm_ws/src/zeroerr_arm/zeroerr_test/" + file_name

        self.get_logger().info(f"Saving {file_name} into {file_path}")

        with open(file_path, 'wb') as file:
            pickle.dump(scene, file, protocol=pickle.HIGHEST_PROTOCOL)

        self.get_logger().info("Saved!")
        

    
    def load_test(self):

        scene_name = input("Name of scene to load: ")

        file_name = scene_name + ".scene"
        file_path = "/home/arodev0/arm_ws/src/zeroerr_arm/zeroerr_test/" + file_name

        scene = PlanningScene()
        try:
            # pickled_scene = open(file_path, 'rb')
            with open(file_path, 'rb') as handle:
                scene = pickle.load(handle)
            # pickled_scene.close()
        except Exception as e:
            self.get_logger().error(f"Exception caught when loading file: {e}")
            return

        aps_request = ApplyPlanningScene.Request()
        aps_request.scene = scene

        future_response = self.apply_ps_client_.call_async(aps_request)

        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result().success:
            self.get_logger().info(f"Updated PlanningScene with new object!")
        else:
            self.get_logger().error(f"Failed to update PlanningScene")




def main(args=None):

    rclpy.init(args=args)


    node = CollisionObjectTest()


    try:
        while True:
            print("""\nEnter number/phrase corresponding to an action:
            '1': Example: add table
            '2': Example: remove table
            'save': Save the current scene
            'load': Load a saved scene
            """)
            try:
                test_no = input()
            except:
                node.destroy_node()
                rclpy.shutdown()


            if test_no == '1':
                node.add_table()
            elif test_no == '2':
                node.remove_table()
            elif test_no == 'save':
                node.save_test()
            elif test_no == 'load':
                node.load_test()

    except:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()