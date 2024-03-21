import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from zeroerr_msgs.msg import JointSpaceTarget, PoseTarget

class MoveGroupTest(Node):

    def __init__(self):
        super().__init__('move_group_test')
        
        # JointSpaceTarget publisher
        self.joint_space_target_pub_ = self.create_publisher(
            JointSpaceTarget,
            "arm/JointSpaceGoal",
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
            'h': Home test
            """)
        test_no = input()

        if test_no == '1':
            self.basic_move_test_()
        elif test_no == '2':
            self.stop_test_()
        elif test_no == 'h':
            self.home_test_()


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

                    jst.speed = 15
                    jst.joint_deg.


            self.get_logger().info("Stop test finished!")
        

        else:
            self.get_logger().info("Exiting stop test.")
            return



    def home_test_(self):
        self.get_logger().info('Homing!')

        jst = JointSpaceTarget()
        jst.speed = 10
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