#!/usr/bin/env python3

import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from src.publisher import ImagePublisher
from src.subscriber import ImageSubscriber


class ImageHandling(Node):
    def __init__(self):
        super().__init__('image_handling')

        self.picture_directory = str(Path.home()) + '/hsrb-display/hsrb-display/display_images/'
        self.default_picture = os.path.join(self.picture_directory, 'default_logo.png')
        self.doing_picture = os.path.join(self.picture_directory, 'detecting.png')

        # Publisher erstellen
        self.pub = ImagePublisher()

        # Subscriber optional:
        # self.sub = ImageSubscriber()
        # time.sleep(3)

        self.run_sequence()

    def run_sequence(self):
        # 1. Default anzeigen
        self.pub.publish_image(self.default_picture)
        time.sleep(3)

        # 2. Detecting anzeigen
        self.pub.publish_image(self.doing_picture)
        time.sleep(3)

        # 3. Wieder zurück auf Default
        self.pub.publish_image(self.default_picture)

        self.get_logger().info("done")


def main(args=None):
    rclpy.init(args=args)

    node = ImageHandling()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
