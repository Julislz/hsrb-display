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
        # package_share = get_package_share_directory('hsrb-display')

        # Path to the folder that contains the display images
        # self.media_dir = os.path.join(package_share, "display_images")
        
        # hardcoded path
        self.media_dir = "/home/administrator/hsr_display_ws/src/hsrb_display/hsrb_display/display_images"


        # Mapping: ID number to image file name
        self.media_map = {
            0: "default_logo.png",
            # add new mappings
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
        text = msg.data
        self.get_logger().info(f"Received text: {text}")

        # Create blank image
        img = np.zeros((600, 800, 3), dtype=np.uint8)

        # Word wrapping
        max_chars = 20
        lines = [text[i:i + max_chars] for i in range(0, len(text), max_chars)]

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 2.0
        thickness = 3
        color = (255, 255, 255)

        # Starting vertical position
        y = 150

        for line in lines:
            cv2.putText(
                img,
                line,
                (50, y),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA
            )
            y += 90  # line height spacing

        # Publish image
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
