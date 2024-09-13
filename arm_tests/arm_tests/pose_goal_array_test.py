from os import posix_fadvise
import time

import numpy as np

import copy
from tracemalloc import start

from py import test

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from std_srvs.srv import Trigger
from arm_msgs.srv import PoseGoalArray, PoseGoal, JointSpaceGoal


class PoseGoalArrayTest(Node):
        
    def __init__(self):
        super().__init__('pga_test_node')

        # Execute feature client
        self.execute_client_ = self.create_client(
            Trigger,
            "arm/Execute")
        
        self.execute_rq = Trigger.Request()
        
        while not self.execute_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Execute service not available, waiting again...')


        # PoseGoal feature client
        self.pg_client_ = self.create_client(
            PoseGoal, 
            "arm/PoseGoal")

        while not self.pg_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PoseGoal service not available, waiting again...')
        

        # JointSpaceGoal feature client
        self.jsg_client_ = self.create_client(
            JointSpaceGoal, 
            "arm/JointSpaceGoal")

        while not self.jsg_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('JointSpaceGoal service not available, waiting again...')


        # PoseGoalArray feature client
        self.pga_client_ = self.create_client(
            PoseGoalArray, 
            "arm/PoseGoalArray")

        self.pga_rq = PoseGoalArray.Request()

        while not self.pga_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('PoseGoalArray service not available, waiting again...')



    def linear_test(self):

        home_pose_request = JointSpaceGoal.Request()
        home_pose_request.joint_pos_deg.fill(0)
        home_pose_request.speed = 50

        js_request = JointSpaceGoal.Request()
        js_request.speed = 35
        js_request.joint_pos_deg[0] = 90
        js_request.joint_pos_deg[1] = -15
        js_request.joint_pos_deg[2] = 75
        js_request.joint_pos_deg[3] = 90
        js_request.joint_pos_deg[4] = 90
        js_request.joint_pos_deg[5] = 0

        start_pose = Pose()
        start_pose.position.x = -0.4717
        start_pose.position.y = 0.0252
        start_pose.position.z = 0.6079
        start_pose.orientation.x = 0.7071
        start_pose.orientation.y = 0.0
        start_pose.orientation.z = 0.7071
        start_pose.orientation.w = 0.0
        


        input("Enter any key to move to start pose\n")

        # Generate motion plan to start pose
        future_response = self.jsg_client_.call_async(js_request)

        # Wait for result
        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info("Motion plan successful!\n")
        else:
            self.get_logger().error("Motion plan failed\n")
            return


        # Execute motion plan to start pose
        input("Enter any key to execute\n")
        self.execute_client_.call_async(self.execute_rq)
        

        
        input("Enter any key to begin yz-plane rectangle\n")

        # Generate cartesian motion plan
        pga_request = PoseGoalArray.Request()

        pga_request.type = "linear"
        pga_request.step_size = 0.005

        start_pose.position.z -= 0.2
        waypoint_to_add = copy.deepcopy(start_pose)
        pga_request.waypoints.append(waypoint_to_add)

        start_pose.position.y += 0.15
        waypoint_to_add = copy.deepcopy(start_pose)
        pga_request.waypoints.append(waypoint_to_add)

        start_pose.position.z += 0.3
        waypoint_to_add = copy.deepcopy(start_pose)
        pga_request.waypoints.append(waypoint_to_add)

        start_pose.position.y -= 0.3
        waypoint_to_add = copy.deepcopy(start_pose)
        pga_request.waypoints.append(waypoint_to_add)

        start_pose.position.z -= 0.3
        waypoint_to_add = copy.deepcopy(start_pose)
        pga_request.waypoints.append(waypoint_to_add)

        self.get_logger().info("Making service call...")
        future_response = self.pga_client_.call_async(pga_request)

        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info("PoseGoalArray service successful!")
        else:
            self.get_logger().error("PoseGoalArray service failed")
            return
        

        # Execute cartesian path
        if input("Execute (y/n)?").lower() == 'y':
            self.execute_client_.call_async(self.execute_rq)
        else:
            self.get_logger().error("Test aborted")
            return


        # Move to home position
        input("Enter any key to home arm\n")
        future_response = self.jsg_client_.call_async(home_pose_request)

        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info("JointSpaceGoal service successful!")
        else:
            self.get_logger().error("JointSpaceGoal service failed")
            return


        # Execute motion plan to home pose
        if input("Execute (y/n)?").lower() == 'y':
            self.execute_client_.call_async(self.execute_rq)
        else:
            self.get_logger().error("Test aborted")
            return
        


        # input("Enter any key to move to start pose\n")

        # start_pose.orientation.x = -0.00040959042962640524
        # start_pose.orientation.y = -0.00035000048228539526
        # start_pose.orientation.z = -0.7071999907493591
        # start_pose.orientation.w = 0.7070133686065674
        # start_pose.position.x = 0.38288721442222595
        # start_pose.position.y = -0.20574696362018585
        # start_pose.position.z = 0.7797312140464783

        # pg_request.pose = start_pose
        # pg_request.speed = 45


        # # Generate motion plan to start pose
        # future_response = self.pg_client_.call_async(pg_request)

        # # Wait for result
        # rclpy.spin_until_future_complete(self, future_response)

        # if future_response.result:
        #     self.get_logger().info("Motion plan successful!\n")
        # else:
        #     self.get_logger().error("Motion plan failed\n")
        #     return


        # # Execute motion plan to start pose
        # input("Enter any key to execute\n")
        # self.execute_client_.call_async(self.execute_rq)



        # input("Enter any key to begin xz-plane rectangle\n")

        # # Generate cartesian motion plan
        # pga_request = PoseGoalArray.Request()
        # pga_request.type = "linear"

        # start_pose.position.x -= 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.y -= 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.x += 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.y -= 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.x -= 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.y -= 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.x += 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.y += 0.4
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.z += 0.12
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.y -= 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)

        # start_pose.position.z -= 0.1
        # waypoint_to_add = copy.deepcopy(start_pose)
        # pga_request.waypoints.append(waypoint_to_add)


        # self.get_logger().info("Making service call...\n")
        # future_response = self.pga_client_.call_async(pga_request)

        # rclpy.spin_until_future_complete(self, future_response)

        # if future_response.result:
        #     self.get_logger().info("PoseGoalArray service successful!\n")
        # else:
        #     self.get_logger().error("PoseGoalArray service failed")
        #     return
        

        # # Execute cartesian path
        # if input("Execute (y/n)?\n").lower() == 'y':
        #     self.execute_client_.call_async(self.execute_rq)
        # else:
        #     self.get_logger().error("Test aborted")
        #     return


        # # Move to home position
        # input("Enter any key to home arm\n")
        # future_response = self.jsg_client_.call_async(home_pose_request)

        # rclpy.spin_until_future_complete(self, future_response)

        # if future_response.result:
        #     self.get_logger().info("JointSpaceGoal service successful!\n")
        # else:
        #     self.get_logger().error("JointSpaceGoal service failed")
        #     return


        # # Execute motion plan to home pose
        # if input("Execute (y/n)?\n").lower() == 'y':
        #     self.execute_client_.call_async(self.execute_rq)
        # else:
        #     self.get_logger().error("Test aborted")
        #     return


        # self.get_logger().info("Linear test complete\n")


    def arc_test(self):

        pga_request = PoseGoalArray.Request()
        pga_request.type = "arc"

        home_pose_request = JointSpaceGoal.Request()
        home_pose_request.joint_pos_deg.fill(0)

        start_pose = Pose()
        start_pose.orientation.w = 1.0
        start_pose.orientation.x = 0.0
        start_pose.orientation.y = 0.0
        start_pose.orientation.z = 0.0
        start_pose.position.x = 0.14437
        start_pose.position.y = 0.081665
        start_pose.position.z = 0.77396

        pg_request = PoseGoal.Request()
        pg_request.pose = start_pose
        pg_request.speed = 45

        
        input("\nEnter any key to move to start pose\n")

        # Generate motion plan to start pose
        future_response = self.pg_client_.call_async(pg_request)

        # Wait for result
        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info("Motion plan successful!\n")
        else:
            self.get_logger().error("Motion plan failed\n")
            return


        # Execute motion plan to start pose
        if input("Execute (y/n)?\n").lower() == 'y':
            self.execute_client_.call_async(self.execute_rq)
        else:
            self.get_logger().error("Test aborted")
            return


        # #* Quarter-circle arc
        radius = 0.1 # 10cm
        center = Pose()
        center.position.x = start_pose.position.x - radius
        center.position.y = start_pose.position.y
        center.position.z = start_pose.position.z 
        pga_request.waypoints.append(center)

        endpoint = Pose()
        endpoint.position.x = start_pose.position.x - (radius * (1 + (np.cos(np.pi/4))))
        endpoint.position.y = start_pose.position.y
        endpoint.position.z = start_pose.position.z + (radius * (np.sin(np.pi/4)))
        pga_request.waypoints.append(endpoint)


        input("\nEnter any key to generate arc motion plan ")
        self.get_logger().info("\nMaking service call...")

        future_response = self.pga_client_.call_async(pga_request)

        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info("PoseGoalArray service successful!\n")
        else:
            self.get_logger().error("PoseGoalArray service failed")
            return
        

        # Execute cartesian path
        if input("Execute (y/n)?").lower() == 'y':
            self.execute_client_.call_async(self.execute_rq)
        else:
            self.get_logger().error("Test aborted")
            return


         # Move to home position
        input("Enter any key to home arm\n")
        future_response = self.jsg_client_.call_async(home_pose_request)

        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info("JointSpaceGoal service successful!")
        else:
            self.get_logger().error("JointSpaceGoal service failed")
            return


        # Execute motion plan to home pose
        if input("Execute (y/n)?").lower() == 'y':
            self.execute_client_.call_async(self.execute_rq)
        else:
            self.get_logger().error("Test aborted")
            return

        self.get_logger().info("\nArc test completed.\n\n\n")



def main(args=None):
    rclpy.init(args=args)

    node = PoseGoalArrayTest()

    while True:
        print("""\nEnter number/character corresponding to an action:
        '1': Linear movement test
        '2': Arc movement test
        'h': Home
        'c': Setup lab environment
        """)
        test_no = input()

        if test_no == '1':
            node.linear_test()
        elif test_no == '2':
            node.arc_test()


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()