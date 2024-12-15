import rclpy
from rclpy.node import Node, ParameterDescriptor
from rclpy.parameter import ParameterType
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from std_msgs.msg import String
from std_msgs.msg import Int64


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.declare_parameter(
            name="namespace",
            value="/tb3_0",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Namespace for the camera and publisher topic, should correspond to a turtlebot",
            ),
        )
        self.namespace = (
            self.get_parameter("namespace").get_parameter_value().string_value
        )
        self.declare_parameter(
            name="preview",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Boolean True/False whether to show detection preview window, default True",
            ),
        )
        self.show_preview = (
            self.get_parameter("preview").get_parameter_value().bool_value
        )
        self.get_logger().info(
            f"Aruco detector is running on namespace {self.namespace}"
        )
        self.camera_sub = self.create_subscription(
            Image,
            self.namespace + "/camera/image_raw",
            self.image_callback,
            10,
        )
        self.camera_sub  # prevent unused variable warning
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.result_pub = self.create_publisher(
            Int64, self.namespace + "/aruco_marker_scan", 10
        )

        self.previous_scan = -1

    def image_callback(self, msg):
        try:
            # Convert to OpenCV image, internet recomended mono8 encoding
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

            # Do the ArUco detection
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is not None:
                id = int(ids.flatten()[0])
                if id != self.previous_scan:
                    self.get_logger().info(f"Detected markers: {id}")
                    msg = Int64()
                    msg.data = id
                    self.result_pub.publish(msg)
                    self.previous_scan = id
            # else:
            #     self.get_logger().info("No markers detected")

            if self.show_preview:
                if ids is not None:
                    # Draw detected marker on image
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                # Show detected marker in image preview
                cv2.imshow("ArUco Detection", cv_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    controller = ArucoDetector()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
