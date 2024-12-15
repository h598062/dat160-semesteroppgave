import rclpy
from rclpy.node import Node


class FinalIteration(Node):
    def __init__(self):
        super().__init__("final_iteration")
        self.get_logger().info("Final Iteration of robot challenge solver started.")
        self.wall_follow_srv = self.create_client()


def main(args=None):
    rclpy.init(args=args)
    controller = FinalIteration()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
