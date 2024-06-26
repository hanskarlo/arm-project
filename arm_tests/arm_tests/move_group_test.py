import time
import copy
import yaml

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

from arm_msgs.msg import CollisionObject
from arm_msgs.msg import JointSpaceTarget
from arm_msgs.msg import PoseTargetArray
from arm_msgs.msg import PoseTarget
from arm_msgs.srv import JointSpaceGoal


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


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

        # JointSpaceGoal client 
        self.jsg_client = self.create_client(JointSpaceGoal, "Joint Space Goal Service")

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
        print("""\nEnter number/character corresponding to an action:
            '1': Basic move test
            '2': Stop test
            '3': Linear movement test
            '4': Arc test
            '5': Precision test
            'h': Home test
            'c': Setup lab environment
            """)
        test_no = input()

        if test_no == '1':
            self.basic_move_test_()
        elif test_no == '2':
            self.stop_test_()
        elif test_no == '3':
            self.linear_test_()
        elif test_no == '4':
            self.arc_test_()
        elif test_no == '5':
            self.precision_test_()
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

                if i < 4:
                    jst.joint_deg[i] = 90
                else:
                    jst.joint_deg[i] = 180
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
        jst.speed = 50
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


    def linear_test_(self):

        msg = Bool()
        msg.data = True

        start_pose = Pose()
        start_pose.orientation.w = 1.0
        start_pose.orientation.x = 0.0
        start_pose.orientation.y = 0.0
        start_pose.orientation.z = 0.0
        start_pose.position.x = 0.14437
        start_pose.position.y = 0.081665
        start_pose.position.z = 0.77396

        #* Go to start pose
        pt = PoseTarget()
        pt.speed = 45
        pt.pose = start_pose

        self.get_logger().info("Moving to starting point")

        self.pose_target_pub_.publish(pt)
        self.execute_pub_.publish(msg)

        time.sleep(10)

        if (input("Start rectangle in xz plane (y/n)?").lower() == 'y'):

            #* Rectangle in xz plane
            pta = PoseTargetArray()
            pta.type = "linear"
            pta.step_size = 0.01     # 1cm step size interpolation
            pta.jump_threshold = 0.0 # Disabled
            start_pose.position.x += 0.2
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.z += 0.15
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.x -= 0.3
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.z -= 0.15
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)


            self.pose_array_pub_.publish(pta)

            if input("Execute (y/n)?").lower() == 'y':
                self.execute_pub_.publish(msg)
            else:
                self.clear_pub_.publish(msg)
                return
        
        else:
            self.clear_pub_.publish(msg)
            return
        
        if (input("Bring arm home (y/n)?").lower() == 'y'):
            self.home_test_()
        else:
            return

        if (input("Start rectangle in yz plane (y/n)?").lower() == 'y'):
            
            #* New start pos in yz plane
            start_pose.orientation.x = -0.00040959042962640524
            start_pose.orientation.y = -0.00035000048228539526
            start_pose.orientation.z = -0.7071999907493591
            start_pose.orientation.w = 0.7070133686065674
            start_pose.position.x = 0.38288721442222595
            start_pose.position.y = -0.20574696362018585
            start_pose.position.z = 0.7797312140464783

            pt = PoseTarget()
            pt.speed = 45
            pt.pose = start_pose

            self.get_logger().info("Moving to starting point")
            self.pose_target_pub_.publish(pt)
            self.execute_pub_.publish(msg)
            time.sleep(15)

            #* Rectangle (10cm x 10cm) in xz plane
            pta = PoseTargetArray()
            pta.type = "linear"
            pta.step_size = 0.02     # 1cm step size interpolation
            pta.jump_threshold = 0.0 # Disabled

            start_pose.position.x -= 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.y -= 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.x += 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.y -= 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.x -= 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.y -= 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.x += 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.y += 0.4
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.z += 0.12
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.y -= 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            start_pose.position.z -= 0.1
            waypoint_to_add = copy.deepcopy(start_pose)
            pta.waypoints.append(waypoint_to_add)

            self.pose_array_pub_.publish(pta)


            if input("Execute (y/n)?").lower() == 'y':
                self.execute_pub_.publish(msg)
            else:
                self.clear_pub_.publish(msg)
                return
        
        else:
            self.clear_pub_.publish(msg)
            return


    def arc_test_(self):

        msg = Bool()
        msg.data = True

        pta = PoseTargetArray()
        pta.type = "arc"
        pta.speed_factor = 10

        start_pose = Pose()
        start_pose.orientation.w = 1.0
        start_pose.orientation.x = 0.0
        start_pose.orientation.y = 0.0
        start_pose.orientation.z = 0.0
        start_pose.position.x = 0.14437
        start_pose.position.y = 0.081665
        start_pose.position.z = 0.77396

        #* Go to start pose
        pt = PoseTarget()
        pt.speed = 45
        pt.pose = start_pose

        self.get_logger().info("Moving to starting point")

        self.pose_target_pub_.publish(pt)
        self.execute_pub_.publish(msg)

        time.sleep(15)

        #* Quarter-circle arc
        radius = 0.1
        center = Pose()
        center.position.x = start_pose.position.x - radius
        center.position.y = start_pose.position.y
        center.position.z = start_pose.position.z 
        pta.waypoints.append(center)

        endpoint = Pose()
        endpoint.position.x = start_pose.position.x - radius
        endpoint.position.y = start_pose.position.y
        endpoint.position.z = start_pose.position.z + radius
        pta.waypoints.append(endpoint)
        pta.speed_factor = 5

        self.pose_array_pub_.publish(pta)


    def precision_test_(self):
        print("\nMeasure a coordinate in 3D space using the reference bolt, where")
        print("+x points to the left of table, +y points towards the desk, +z points toward the ceiling\n")


        speed = int(input("Speed (1-100%)?: "))
        x = float(input("Input x coordinate: "))
        y = float(input("Input y coordinate: "))
        z = float(input("Input z coordinate: "))

        roll = np.deg2rad(int(input("Roll (in degrees)?:  ")))
        pitch = np.deg2rad(int(input("Pitch (in degrees)?: ")))
        yaw = np.deg2rad(int(input("Yaw (in degrees)?:   ")))


        p = PoseTarget()
        p.speed = speed
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z # Begin 10cm above target

        [p.pose.orientation.x, 
         p.pose.orientation.y, 
         p.pose.orientation.z, 
         p.pose.orientation.w] = get_quaternion_from_euler(roll, pitch, yaw)


        #* Offsets from base_link to reference bolt
        x_offset = 0.031
        y_offset = -0.031
        z_offset = 0.014

        p.pose.position.x += x_offset
        p.pose.position.y += y_offset
        p.pose.position.z += z_offset


        #* Create motion plan
        self.pose_target_pub_.publish(p)

        msg = Bool()
        msg.data = True
        if input("Execute (y/n)?").lower() == 'y':
            self.execute_pub_.publish(msg)
        else:
            self.clear_pub_.publish(msg)
            return


        if input("Continue (y/n)?").lower() == 'y':
            pta = PoseTargetArray()
            pta.type = "linear"
            pta.step_size = 0.01
            pta.jump_threshold = 0.0

            pta.waypoints.append(copy.deepcopy(p.pose))

            p.pose.position.z -= 0.1
            pta.waypoints.append(copy.deepcopy(p.pose))

            p.pose.position.z += 0.1
            pta.waypoints.append(copy.deepcopy(p.pose))

            self.pose_array_pub_.publish(pta)
        else:
            self.clear_pub_.publish(msg)
            return
        

        if input("Execute (y/n)?").lower() == 'y':
            self.execute_pub_.publish(msg)
        else:
            self.clear_pub_.publish(msg)
            return
    

        if input("Continue (y/n)?").lower() == 'y':
            p.pose.position.x += 0.1
            self.pose_target_pub_.publish(p)
        else:
            self.clear_pub_.publish(msg)
            return


        if input("Execute (y/n)?").lower() == 'y':
            self.execute_pub_.publish(msg)
        else:
            self.clear_pub_.publish(msg)
            return
        

        if input("Continue (y/n)?").lower() == 'y':
            pta = PoseTargetArray()
            pta.type = "linear"
            pta.step_size = 0.01
            pta.jump_threshold = 0.0

            pta.waypoints.append(copy.deepcopy(p.pose))

            p.pose.position.z -= 0.1
            pta.waypoints.append(copy.deepcopy(p.pose))

            p.pose.position.z += 0.1
            pta.waypoints.append(copy.deepcopy(p.pose))

            self.pose_array_pub_.publish(pta)
        else:
            self.clear_pub_.publish(msg)
            return


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