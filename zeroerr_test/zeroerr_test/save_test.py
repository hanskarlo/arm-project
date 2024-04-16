import yaml
from yaml import Loader
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from zeroerr_msgs.msg import JointSpaceTarget

SAVE_DIR = "/home/aroarm0/arm_ws/src/zeroerr_arm/zeroerr_test/saved/"

class SaveTest(Node):
        
    def __init__(self):
        super().__init__('save_traj_node')

        # Trajectory subscriber
        self.traj_sub_ = self.create_subscription(
            JointTrajectory,
            "arm/SaveTrajectory",
            self.save_joint_trajectory_,
            10
        )

        # Pose subscriber
        self.pose_sub_ = self.create_subscription(
            JointSpaceTarget,
            "arm/SavePose",
            self.save_pose_,
            10
        )


    def save_pose_(self, pose: JointSpaceTarget):

        with open(SAVE_DIR + "poses.yaml", 'a') as file:
            yaml.dump(pose, file)

        with open(SAVE_DIR + "poses.yaml", 'r') as file:
            jst: JointSpaceTarget = yaml.load(file, Loader)

            print(jst)


    def save_joint_trajectory_(self, joint_trajectory: JointTrajectory):
        print(f'Saving trajectory with {len(joint_trajectory.points)} points into JointTrajectory.yaml')

        with open('JointTrajectory.yaml', 'w') as outfile:
            yaml.dump(joint_trajectory, outfile, default_flow_style=False)


def main(args=None):
    rclpy.init(args=args)

    node = SaveTest()

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Caught exception during spin: {e}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()