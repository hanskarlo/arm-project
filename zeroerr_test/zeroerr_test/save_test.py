import rclpy
from rclpy.node import Node

from zeroerr_msgs.srv import Save

SAVE_DIR = "/home/aroarm0/arm_ws/src/zeroerr_arm/zeroerr_test/saved/"

class SaveTest(Node):
        
    def __init__(self):
        super().__init__('save_test_node')

        # Save feature client
        self.save_client_ = self.create_client(Save, "arm/Save")

        while not self.save_client_.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Save service not available, waiting again...')

        rq = Save.Request()
        rq.type = input("Saving 'pose' or 'trajectory'?: ")
        rq.label = input("Label for pose/trajectory?: ")

        self.get_logger().info("Making service call...")
        future_response = self.save_client_.call_async(rq)

        rclpy.spin_until_future_complete(self, future_response)

        if future_response.result:
            self.get_logger().info("Save service successful!")
        else:
            self.get_logger().error("Save service failed")
        


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