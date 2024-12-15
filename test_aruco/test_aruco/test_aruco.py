import rclpy
from rclpy.node import Node, ParameterDescriptor
from rclpy.parameter import ParameterType
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from std_msgs.msg import String


class ArucoTest(Node):
    def __init__(self):
        super().__init__("aruco")
        self.declare_parameter(
            name="namespace",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Namespace for the camera and publisher topic, should correspond to a turtlebot",
            ),
        )
        self.camera_topic = (
            self.get_parameter("namespace").get_parameter_value().string_value
        )
        self.get_logger().info("Hello from aruco")
        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic + "/camera/image_raw",
            self.image_callback,
            10,
        )
        self.camera_sub  # prevent unused variable warning
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.result_pub = self.create_publisher(
            String, self.camera_topic + "/aruco_marker_scan", 10
        )

    def image_callback(self, msg):
        try:
            # Convert to OpenCV image, internet recomended mono8 encoding
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

            # Do the ArUco detection
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is not None:
                id = ids.flatten()[0]
                self.get_logger().info(f"Detected markers: {id}")
                # Draw detected marker on image
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                msg = String()
                msg.data = f"{id}"
                self.result_pub.publish(msg)
            else:
                self.get_logger().info("No markers detected")

            # Show detected marker in image preview
            cv2.imshow("ArUco Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    controller = ArucoTest()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
