from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node, ParameterDescriptor
from rclpy.parameter import ParameterType
from std_srvs.srv import SetBool
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import qos_profile_sensor_data


class FinalIteration(Node):
    def __init__(self, name="final_iteration"):
        super().__init__(name)
        # create and use parameter for namespace, tb3_0 is default for easier testing
        self.declare_parameter(
            name="namespace",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Namespace for the program for findinf correct topics, should correspond to turtlebot namespace",
            ),
        )
        self.namespace = (
            self.get_parameter("namespace").get_parameter_value().string_value
        )
        self.get_logger().info(
            f"Final Iteration of solver started, using namespace {self.namespace}."
        )
        self.wall_follow_srv = self.create_client(
            SetBool, self.namespace + "/wall_follower_service"
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.namespace + "/odom", self.odom_clbk, 10
        )
        self.marker_sub = self.create_subscription(
            Int64,
            self.namespace + "/aruco_marker_scan",
            self.marker_clbk,
            qos_profile_sensor_data,
        )

        self.position: Point = Point()
        self.last_marker = -1
        self.found_markers = {}  # store found markers in a dict, position as key, rounded to whole integer position

        while not self.wall_follow_srv.wait_for_service(15):
            self.get_logger().info("Wall follower service is not available, waiting...")

        msg = SetBool.Request()
        msg.data = True
        self.wall_follow_srv.call_async(msg)

    def odom_clbk(self, msg: Odometry):
        self.position = msg.pose.pose.position

    def marker_clbk(self, msg: Int64):
        # self.get_logger().info(f"Got marker message {msg}")
        id = msg.data
        if id < 0 or id > 4:  # only ids 0-4 are valid, discard the rest
            return

        # convert position Point to a x,y integer string ex. "1,2". Should be accurate enough
        x = int(self.position.x)
        y = int(self.position.y)
        key = f"{x},{y}"
        if self.found_markers.get(key) is None:
            self.found_markers[key] = id
            self.get_logger().info(f"Found marker at position {key} with id {id}")
            self.last_marker = id


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
