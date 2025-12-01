#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSubscriber(Node):
    def __init__(self, topic="/head_display/receiver"):
        super().__init__("image_subscriber")

        self.bridge = CvBridge()

        # ROS2 subscriber to receive image messages
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"ImageSubscriber started and listening on {topic}")

        # Create a window one time (prevents flickering)
        cv2.namedWindow("HSR Display", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("HSR Display", 800, 600)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            if image is not None:
                cv2.imshow("HSR Display", image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
