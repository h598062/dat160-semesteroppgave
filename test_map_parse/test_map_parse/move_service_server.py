import rclpy
from rclpy.node import Node
from sem_oppg_msgs.srv import MoveSrv
from geometry_msgs.msg import Twist
import time

class MoveServiceServer(Node):
    def __init__(self):
        super().__init__('move_service_server')

        self.srv = self.create_service(MoveSrv, 'move_to_coordinates', self.handle_move_request)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Move Service Server is ready')

    def handle_move_request(self, request, response):
        self.get_logger().info(f'Received move request to x={request.x}, y={request.y}')

        success = self.simulate_movement(request.x, request.y)

        response.result = success

        if success:
            self.get_logger().info(f'Successfully moved to x={request.x}, y={request.y}')
        else:
            self.get_logger().error(f'Failed to move to x={request.x}, y={request.y}')
        return response

    def simulate_movement(self, x, y):
        twist = Twist()
        twist.linear.x = 0.5  # Simulate forward movement
        twist.angular.z = 0.0
        self.publisher.publish(twist)

        # Simulate a delay for moving to the coordinates
        self.get_logger().info(f'Simulating movement to x={x}, y={y}')
        time.sleep(2.0)  # Fake delay (2 seconds)

        # Example logic: always return True for now
        return True

def main(args=None):
    rclpy.init(args=args)
    node = MoveServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
