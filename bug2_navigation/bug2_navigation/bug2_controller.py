import rclpy
from rclpy.node import Node
from bug2_interfaces.srv import PointMover 
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class Bug2Controller(Node):
    def __init__(self):
        super().__init__('bug2_controller')

        self.go_to_point_client = self.create_client(PointMover, 'go_to_point_service')
        self.wall_follower_client = self.create_client(SetBool, 'wall_follower_service')
        
        self.get_logger().info('Venter på GoToPoint service...')
        while not self.go_to_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter fortsatt på GoToPoint service...')
        self.get_logger().info('GoToPoint service tilgjengelig')

        self.get_logger().info('Venter på WallFollower service...')
        while not self.wall_follower_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter fortsatt på WallFollower service...')
        self.get_logger().info('WallFollower service tilgjengelig')
        
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.clbk_laser, 10)

        self.target_position = PoseStamped()
        self.target_position.pose.position.x = 0.0  
        self.target_position.pose.position.y = 7.0

        self.state = 'go_to_point'  
        self.obstacle_detected = False  

        self.allow_switch = True  # To prevent immediate switching between states
        self.switch_to_go_to_point()

    def clbk_laser(self, msg):

        min_distance = min(msg.ranges)
        if min_distance < 0.5:  
            if self.state == 'go_to_point' and self.allow_switch:
                self.get_logger().info('Obstacle detected, switching to wall following.')
                self.allow_switch = False  # Disable immediate switching
                self.switch_to_wall_follower()
        else:
            if self.state == 'wall_following' and not self.obstacle_detected and self.allow_switch:
                self.get_logger().info('Obstacle cleared, switching to go to point.')
                self.allow_switch = False  # Disable immediate switching
                self.switch_to_go_to_point()

    def switch_to_go_to_point(self):
        self.state = 'go_to_point'
        self.allow_switch = True  # Allow future switching

        # Slutt å følge vegg når du bytter til go to point
        wall_follow_request = SetBool.Request()
        wall_follow_request.data = False
        self.wall_follower_client.call_async(wall_follow_request)

        request = PointMover.Request()
        request.move_switch = True
        request.target_position.x = self.target_position.pose.position.x
        request.target_position.y = self.target_position.pose.position.y
        self.go_to_point_client.call_async(request)
        self.get_logger().info('Byttet til GoToPoint.')

    def switch_to_wall_follower(self):
        self.state = 'wall_following'
        self.allow_switch = True  # Allow future switching

        # Stopp gotopoint når du begynner å følge vegg
        go_to_point_request = PointMover.Request()
        go_to_point_request.move_switch = False
        self.go_to_point_client.call_async(go_to_point_request)

        request = SetBool.Request()
        request.data = True
        self.wall_follower_client.call_async(request)
        self.get_logger().info('Byttet til WallFollower.')

def main(args=None):
    rclpy.init(args=args)
    bug2_controller = Bug2Controller()
    rclpy.spin(bug2_controller)
    bug2_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
