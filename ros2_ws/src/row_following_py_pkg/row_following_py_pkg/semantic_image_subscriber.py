import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import os


class SegmentationDisplayNode(Node):
    def __init__(self):
        super().__init__('semantic_image_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/semantic_segmentation',
            self.segmentation_callback,
            10)
        # Define a simple color map: class_id to RGB
        self.colormap = np.array([
            [0, 0, 0],  # Class 0: black Background
            [255, 255, 255],     # Class 1: white Cilantro Om
            [0, 0, 0],       # Class 2: black Ground
        ], dtype=np.uint8)
        self.semantic_img = None
        self.image_dir = '/home/milos/row-following/ros2_ws/Dataset'  # Directory to save images
        self.image_counter = 1
        self.max_images = 300

    def safe_color_mapping(self):
        h, w = self.semantic_img.shape
        color_segmented_image = np.zeros((h, w, 3), dtype=np.uint8)
        
        for i in range(h):
            for j in range(w):
                class_id = self.semantic_img[i, j]
                if class_id < len(self.colormap):
                    color_segmented_image[i, j] = self.colormap[class_id]
                else:
                    # Use a default color if the class ID is out of the colormap's range
                    color_segmented_image[i, j] = [255, 0, 0]  # Example: deep pink for unknown classes
                    self.get_logger().info('Invoked')
        
        return color_segmented_image

    def segmentation_callback(self, msg):

        self.semantic_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32SC1')

        condition = (self.semantic_img == 1) # Class 1: white Cilantro Om
        condition2 = (self.semantic_img == 2) # Class 2: black Ground
        condition3 = (self.semantic_img == 0) # Class 0: black Background
        # print(condition)
        gray_image = np.zeros(self.semantic_img.shape, dtype=np.uint8)
        # rgb_image[condition] = [255, 255, 255]
        # rgb_image[condition2] = [0, 0, 0]
        # rgb_image[condition3] = [0, 0, 0]

        gray_image[condition] = 255
        gray_image[condition2] = 0
        gray_image[condition3] = 0

        cv2.imshow('Segmentation Display', gray_image)
        cv2.waitKey(1)


        # Map the class IDs to colors
        # color_segmentation = self.colormap[self.semantic_img]
        # color_segmentation = self.semantic_img[self.colormap]

        # np.set_printoptions(threshold=np.inf)
        # print(self.semantic_img)

        # color_segmentation = self.safe_color_mapping()
        # m = np.max(self.semantic_img)
        # print(m)

        # if color_segmentation is not None and self.image_counter <= self.max_images:
        #     filename = os.path.join(self.image_dir, f'MaskCilantroOm_{self.image_counter}.png')
        #     cv2.imwrite(filename, color_segmentation)
        #     self.get_logger().info(f'Saved image to {filename}')
        #     self.image_counter += 1
        # else:
        #     self.get_logger().info('No image available to save')

        # Display the color-mapped segmentation image
        # cv2.imshow('Segmentation Display', color_segmentation)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
