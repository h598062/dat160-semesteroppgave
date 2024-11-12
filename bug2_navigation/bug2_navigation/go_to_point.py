import rclpy
from bug2_interfaces.srv import PointMover
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class GoToPoint(Node):

    def __init__(self):
        super().__init__('go_to_point')


        self.get_logger().info("Startet GoToPoint.")

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)
        
        self.srvserver = self.create_service(PointMover, 'go_to_point_service', self.handle_go_to_point)
        
        self.target_position = None
        self.position = None
        self.yaw = None
        self.beveger_mot_posisjon = False
        self.reached_goal = False

    def handle_go_to_point(self, request, response):
        self.beveger_mot_posisjon = request.move_switch
        
        if self.beveger_mot_posisjon:
            
            self.target_position = request.target_position
            self.reached_goal = False  
            self.get_logger().info(f"Ny posisjon: x={self.target_position.x}, y={self.target_position.y}")
        else:
            
            self.target_position = None
            self.get_logger().info("Bevegelse stanset")

        response.success = True  
        return response  

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        
        self.navigate_to_point()

    def navigate_to_point(self):
        if self.position is None or self.yaw is None or self.reached_goal or self.target_position is None:
            return

        # Calculate angle and distance to the target
        target_angle = math.atan2(self.target_position.y - self.position.y,
                                  self.target_position.x - self.position.x)
        distance_to_target = math.sqrt((self.target_position.x - self.position.x) ** 2 +
                                       (self.target_position.y - self.position.y) ** 2)

        twist = Twist()

        if abs(target_angle - self.yaw) > 0.1:
            # Roterer seg mot målet
            twist.angular.z = 0.5 if target_angle > self.yaw else -0.5
        elif distance_to_target > 0.1:
            # Kjører rett fram mot målet
            twist.linear.x = 0.5
        else:
            # Stopper når roboten treffer målet
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.reached_goal = True

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    go_to_point = GoToPoint()
    rclpy.spin(go_to_point)
    go_to_point.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()