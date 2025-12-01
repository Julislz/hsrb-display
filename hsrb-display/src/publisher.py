#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisher(Node):

    def __init__(self, topic="/head_display/receiver"):
        super().__init__('image_publisher')

        self.bridge = CvBridge()

        # ROS2 Publisher
        self.publisher = self.create_publisher(Image, topic, 10)
        self.get_logger().info("Publisher started")

    def publish_image(self, path):
        image = cv2.imread(path)

        if image is not None:
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

            # ROS2 Sleep → Timer / or direct sleep from rclpy
            self.publish_after_delay(1.0, ros_image)
        else:
            self.get_logger().error(f"Failed to load image: {path}")

    def publish_after_delay(self, delay, msg):
        # Timer to publish after a delay
        self.create_timer(delay, lambda: self.publish_and_log(msg))

    def publish_and_log(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info("Published image")
