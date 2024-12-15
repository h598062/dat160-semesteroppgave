import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sem_oppg_msgs.srv import MoveSrv
import math


class MovementController(Node):
    def __init__(self):
        super().__init__("movement_controller")
        self.odom_subscription = self.create_subscription(
            Odometry, "/tb3_0/odom", self.odom_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/tb3_0/cmd_vel", 10)
        self.srv = self.create_service(MoveSrv, "move_to", self.handle_move_request)

        self.robot_position = None
        self.goal_position = None
        self.get_logger().info("Movement controller service started.")

    def odom_callback(self, msg):
        # Update the robot's current position
        self.robot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def handle_move_request(self, request, response):
        if self.robot_position is None:
            self.get_logger().warn("Robot position is not available.")
            response.result = False
            return response

        self.goal_position = (request.x, request.y)
        self.get_logger().info(f"Received move request to: {self.goal_position}")

        # Move toward the goal
        if self.move_toward_goal():
            self.get_logger().info("Goal reached successfully.")
            response.result = True
        else:
            self.get_logger().warn("Failed to reach the goal.")
            response.result = False

        return response

    def move_toward_goal(self):
        goal_x, goal_y = self.goal_position
        while True:
            if self.robot_position is None:
                return False

            robot_x, robot_y = self.robot_position
            distance = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)

            # Check if goal is reached
            if distance < 0.1:
                twist = Twist()
                self.cmd_vel_publisher.publish(twist)  # Stop the robot
                return True

            # Calculate movement commands
            angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
            twist = Twist()
            twist.linear.x = min(0.2, distance)  # Cap linear speed
            twist.angular.z = angle_to_goal
            self.cmd_vel_publisher.publish(twist)

            # Sleep to control the movement frequency
            wait = self.get_clock().now()
            while wait < self.get_clock().now() + 100:
                wait = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    controller = MovementController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        controller.cmd_vel_publisher.publish(Twist())
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
