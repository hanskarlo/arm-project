import time
import copy
import numpy as np
from psutil import wait_procs
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from zeroerr_msgs.msg import CollisionObject
from zeroerr_msgs.msg import JointSpaceTarget
from zeroerr_msgs.msg import PoseTargetArray
from zeroerr_msgs.msg import PoseTarget

class MoveGroupTest(Node):

    def __init__(self):
        super().__init__('move_group_test')
        
        # Collision Object publisher
        # self.collision_obj_pub_ = self.create_publisher(
        #     CollisionObject,
        #     "arm/CollisionObject",
        #     1
        # )
        self.collision_obj_pub_ = self.create_publisher(
            Bool,
            "arm/SetupLabEnvironment",
            1
        )

        # JointSpaceTarget publisher
        self.joint_space_target_pub_ = self.create_publisher(
            JointSpaceTarget,
            "arm/JointSpaceGoal",
            10
        )

        # PoseTargetArray publisher
        self.pose_array_pub_ = self.create_publisher(
            PoseTargetArray,
            "arm/PoseGoalArray",
            10
        )

        # PoseTarget publisher
        self.pose_target_pub_ = self.create_publisher(
            PoseTarget,
            "arm/PoseGoal",
            10
        )

        # Execute motion plan publisher
        self.execute_pub_ = self.create_publisher(
            Bool,
            "arm/Execute",
            1
        )

        # Stop execution publisher
        self.stop_pub_ = self.create_publisher(
            Bool, 
            "arm/Stop",
            1
        )

        # Clear target publisher
        self.clear_pub_ = self.create_publisher(
            Bool,
            "arm/Clear",
            1
        )

        # Loop querying user for test
        self.create_timer(
            0.5,
            self.timer_cb_,
        )


    def timer_cb_(self):
        print("""\nEnter number corresponding to an action:
            '1': Basic move test
            '2': Stop test
            '3': Pose array test
            'h': Home test
            'c': Setup lab environment
            """)
        test_no = input()

        if test_no == '1':
            self.basic_move_test_()
        elif test_no == '2':
            self.stop_test_()
        elif test_no == '3':
            self.pose_array_test_()
        elif test_no == 'h':
            self.home_test_()
        elif test_no == 'c':
            self.collision_object_test_()


    def basic_move_test_(self):
        speed = input("Enter speed (0-100%): ")

        jst = JointSpaceTarget()
        jst.speed = int(speed)
        jst.joint_deg[3] = 45
        jst.joint_deg[4] = 45
        jst.joint_deg[5] = 45

        self.joint_space_target_pub_.publish(jst)

        msg = Bool()
        msg.data = True
        
        if input("Motion plan generated. Execute (y/n)? ").lower() == "y":
            self.get_logger().info("Executing!")
            self.execute_pub_.publish(msg)

        else:
            self.get_logger().info("Execution cancelled, clearing target.")
            self.clear_pub_.publish(msg)


    def stop_test_(self):
        
        cmd = input("Each joint will attempt to move 90° at 30% speed and be stopped after 3s. Continue (y/n)? ")

        if cmd.lower() == 'y':
            msg = Bool()
            msg.data = True

            jst = JointSpaceTarget()

            for i in range(0, 6):
                
                if i < 4:
                    jst.speed = 50
                else:
                    jst.speed = 50
                
                self.get_logger().info(f"Moving J{i} 90° at {jst.speed}% speed")

                jst.joint_deg[i] = 90
                self.joint_space_target_pub_.publish(jst)

                if input("Execute (y/n)? ").lower() == "y":
                    self.get_logger().info("Executing!")
                    self.execute_pub_.publish(msg)

                    ctr = 0
                    stop_time = 0
                    if i < 4:
                        stop_time = 3
                    else:
                        stop_time = 2

                    self.get_logger().info(f"Stopping after {stop_time}s")
                    while True:
                        time.sleep(1)
                        self.get_logger().info(".")
                        
                        ctr += 1
                        if ctr == stop_time:
                            break
                    
                    self.stop_pub_.publish(msg)
                    self.get_logger().info("Arm stopped!")
                    
                    if input("Move joint back home (y/n)?").lower() == 'y':
                        self.get_logger().info("Homing joint.")

                        jst.joint_deg[i] = 0
                        self.joint_space_target_pub_.publish(jst)
                        self.execute_pub_.publish(msg)

                        ctr = 0
                        while True:
                            time.sleep(1)
                            self.get_logger().info(".")
                            
                            ctr += 1
                            if ctr == 5:
                                print('\n')
                                break

                else:
                    self.get_logger().info("Execution cancelled, homing arm.")
                    # self.clear_pub_.publish(msg)

                    jst = JointSpaceTarget()
                    jst.speed = 15
                    self.joint_space_target_pub_.publish(jst)

                    if input("Execute (y/n)?").lower() == 'y':
                        self.get_logger().info("Homing arm.")
                        self.execute_pub_.publish(msg)
                    
                    break


            self.get_logger().info("Stop test finished!")
        

        else:
            self.get_logger().info("Exiting stop test.")
            return


    def home_test_(self):
        self.get_logger().info('Homing!')

        jst = JointSpaceTarget()
        jst.speed = 15
        jst.joint_deg[0] = 0
        jst.joint_deg[1] = 0
        jst.joint_deg[2] = 0
        jst.joint_deg[3] = 0
        jst.joint_deg[4] = 0
        jst.joint_deg[5] = 0
    
        self.joint_space_target_pub_.publish(jst)

        msg = Bool()
        msg.data = True
        if input("Execute (y/n)? ").lower() == "y":
            self.get_logger().info("Executing!")
            self.execute_pub_.publish(msg)

        else:
            self.get_logger().info("Execution cancelled, clearing target.")
            self.clear_pub_.publish(msg)


    def pose_array_test_(self):

        msg = Bool()
        msg.data = True

        pta = PoseTargetArray()
        pta.speed_factor = 25

        waypoint = Pose()
        waypoint.orientation.w = 1.0
        waypoint.orientation.x = 0.0
        waypoint.orientation.y = 0.0
        waypoint.orientation.z = 0.0
        waypoint.position.x = 0.14437
        waypoint.position.y = 0.081665
        waypoint.position.z = 0.77396

        #* Go to start pose
        pt = PoseTarget()
        pt.speed = 30
        pt.pose = waypoint

        self.get_logger().info("Moving to starting point")

        self.pose_target_pub_.publish(pt)
        self.execute_pub_.publish(msg)

        time.sleep(20)


        #* Rectangle
        waypoint.position.x += 0.05
        waypoint_to_add = copy.deepcopy(waypoint)
        pta.waypoints.append(waypoint_to_add)

        waypoint.position.z += 0.05
        waypoint_to_add = copy.deepcopy(waypoint)
        pta.waypoints.append(waypoint_to_add)

        waypoint.position.x -= 0.05
        waypoint_to_add = copy.deepcopy(waypoint)
        pta.waypoints.append(waypoint_to_add)

        waypoint.position.z -= 0.05
        waypoint_to_add = copy.deepcopy(waypoint)
        pta.waypoints.append(waypoint_to_add)


        #* Circle
        # radius = 0.1
        # waypoint_num = 25
        # pta.step_size = 0.001
        # pta.jump_threshold = 0.0
        # arc_length = radius * (np.pi/waypoint_num)
        # x_steps = np.linspace(0, 2*radius, 10)
        # for angle in np.linspace(0, np.pi, waypoint_num + 1):
        #     waypoint.position.x = 0.14437
        #     waypoint.position.y = 0.081665
        #     waypoint.position.z = 0.77396
        #     waypoint.position.x += float(radius*(np.cos(angle) - 1))
        #     waypoint.position.z += float(radius*(np.sin(angle)))

        #     print(f"({waypoint.position.x}, {waypoint.position.z})")

        #     waypoint_to_add = copy.deepcopy(waypoint)
        #     pta.waypoints.append(waypoint_to_add)
    
        
        # center = Pose()
        # center.position.x = waypoint.position.x - radius
        # center.position.y = waypoint.position.y
        # center.position.z = waypoint.position.z 
        # pta.waypoints.append(center)

        # endpoint = Pose()
        # endpoint.position.x = waypoint.position.x - radius
        # endpoint.position.y = waypoint.position.y
        # endpoint.position.z = waypoint.position.z + radius
        # pta.waypoints.append(endpoint)
        # pta.speed_factor = 5


        self.pose_array_pub_.publish(pta)


    def collision_object_test_(self):
        self.get_logger().info("Adding collision object!")

        msg = Bool()
        msg.data = True
        self.collision_obj_pub_.publish(msg)

        # TODO: Collision object support (Adds lab table for now)




def main(args=None):
    rclpy.init(args=args)

    mv_grp_test_node = MoveGroupTest()

    try:
        rclpy.spin(mv_grp_test_node)
    except Exception as e:
        print(f"Caught exception during spin: {e}")

    mv_grp_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()