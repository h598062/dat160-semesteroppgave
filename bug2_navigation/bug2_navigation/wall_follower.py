import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.get_logger().info(' Startet WallFollower.')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Service server
        self.srvserver = self.create_service(SetBool, 'wall_follower_service', self.handle_wall_follower)

        self.wall_distance = 0.5  # Distanse fra veggen
        self.front_distance_threshold = 0.6  # Distanse for å oppdage en vegg foran
        self.turning_speed = 0.5  # Hastighet for å snu seg venstre/høyre
        self.forward_speed = 0.3  # Hastighet for å bevege seg framover
        self.turning_counter = 0  # Liten variabel for å sørge for at roboten klarer å snu seg helt til andre siden av veggen

        # Hardkodet for å følge en vegg
        self.following_wall = False

    def handle_wall_follower(self, request, response):
        self.following_wall = request.data
        response.success = True
        response.message = "Wall follower er nå aktiv" if self.following_wall else "Wall follower er inaktiv"
        return response

    def scan_callback(self, msg):
        if not self.following_wall:
            return
        
        front_distance = min(msg.ranges[0:20] + msg.ranges[340:359])
        right_distance = min(msg.ranges[260:300])

        twist = Twist()

        # Hvis vegg er oppdaget foran, snu seg til høyre
        if front_distance < self.front_distance_threshold:
            twist.linear.x = 0.0  
            twist.angular.z = self.turning_speed 
            self.following_wall = True

        # Veggen er nå til venstre, følg den
        elif right_distance < self.wall_distance:
            twist.linear.x = self.forward_speed  
            twist.angular.z = -0.1  

        # Veggen slutter, flytt deg til andre siden
        elif right_distance > self.wall_distance:
            if self.turning_counter < 5:  # Delay slik at roboten kan snu seg fullt
                twist.linear.x = 0.2
                twist.angular.z = -self.turning_speed 
                self.turning_counter += 1  # Increment
            else:
                self.turning_counter = 0  # Reset counter

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
