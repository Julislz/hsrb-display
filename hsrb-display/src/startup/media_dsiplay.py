#!/usr/bin/env python3

import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class MediaDisplay(Node):

    def __init__(self):
        # Initialize the ROS2 node
        super().__init__("media_display")

        # Bridge to convert between ROS2 Image messages and OpenCV images
        self.bridge = CvBridge()

        # Get the package share directory where images are stored
        package_share = get_package_share_directory('hsrb-display')

        # Path to the folder that contains the display images
        self.media_dir = os.path.join(package_share, "display_images")

        # Mapping: ID number to image file name
        self.media_map = {
            0: "walle.png",
            1: "default_logo.png",
            # You can extend this dictionary with more entries
        }

        # Publisher that sends images to the HSR head display
        self.image_pub = self.create_publisher(Image, "/head_display/receiver", 10)

        # Subscriber that receives text and converts it into an image
        self.create_subscription(
            String,
            "/head_display/text_to_image",
            self.text_callback,
            10
        )

        # Subscriber that receives an ID to switch between predefined images
        self.create_subscription(
            Int32,
            "/media_switch_topic",
            self.media_callback,
            10
        )

        self.get_logger().info("MediaDisplay node started (ROS2)")

    # -------------------------------------------------------------------------
    # TEXT TO IMAGE CONVERSION
    # -------------------------------------------------------------------------

    def text_callback(self, msg):
        # Extract the received text
        text = msg.data
        self.get_logger().info(f"Received text: {text}")

        # Create a blank image (black background)
        img = np.zeros((600, 800, 3), dtype=np.uint8)

        # Add the text onto the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(
            img,
            text,
            (50, 300),      # text position
            font,
            1.2,            # font scale
            (255, 255, 255),# white color
            2,              # thickness
            cv2.LINE_AA     # anti-aliased line
        )

        # Publish the generated OpenCV image as a ROS2 Image message
        self.publish_opencv_image(img)

    # -------------------------------------------------------------------------
    # MEDIA SWITCH HANDLING
    # -------------------------------------------------------------------------

    def media_callback(self, msg):
        # Extract the received media ID
        media_id = msg.data
        self.get_logger().info(f"Media switch request: ID {media_id}")

        # Check whether the ID exists in the dictionary
        if media_id not in self.media_map:
            self.get_logger().error(f"Unknown media ID: {media_id}")
            return

        # Construct the full path to the image file
        filename = os.path.join(self.media_dir, self.media_map[media_id])

        # Load the image using OpenCV
        img = cv2.imread(filename)
        if img is None:
            self.get_logger().error(f"Could not load image: {filename}")
            return

        # Publish the loaded image to the robot display
        self.publish_opencv_image(img)

    # -------------------------------------------------------------------------
    # HELPER FUNCTION: Convert OpenCV image to ROS2 Image message
    # -------------------------------------------------------------------------

    def publish_opencv_image(self, img):
        # Convert OpenCV image to ROS Image message
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")

        # Publish the ROS2 Image message
        self.image_pub.publish(ros_img)

        self.get_logger().info("Displayed image published.")

# -------------------------------------------------------------------------
# MAIN FUNCTION
# -------------------------------------------------------------------------

def main(args=None):
        # Initialize ROS2 client library
        rclpy.init(args=args)

        # Create the MediaDisplay node
        node = MediaDisplay()

        # Keep the node running and processing callbacks
        rclpy.spin(node)

        # Clean up on shutdown
        node.destroy_node()
        rclpy.shutdown()


# Entry point
if __name__ == "__main__":
    main()
