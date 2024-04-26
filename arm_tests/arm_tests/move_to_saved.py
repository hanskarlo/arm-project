import rclpy
from rclpy.node import Node

from arm_msgs.srv import MoveToSaved

SAVE_DIR = "/home/aroarm0/arm_ws/src/zeroerr_arm/zeroerr_test/saved/"

class MoveToSavedTest(Node):
        
    def __init__(self):
        super().__init__('move_to_saved_test_node')

        # Save feature client
        self.client_ = self.create_client(MoveToSaved, "arm/ExecuteSaved")

        while not self.client_.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Save service not available, waiting again...')

        self.rq = MoveToSaved.Request()

    def send_request(self):
        self.rq.type = input("'pose' or 'trajectory'? ")
        self.rq.label = input(f"Input {self.rq.type} label: ")

        self.get_logger().info("Making service call...")
        future_response = self.client_.call_async(self.rq)

        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info("ExecuteSaved service successful!")
        else:
            self.get_logger().error("ExecuteSaved service failed")
        


def main(args=None):
    rclpy.init(args=args)

    node = MoveToSavedTest()

    node.send_request()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()