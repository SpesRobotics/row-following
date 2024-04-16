import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

import numpy as np
import os

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.image_subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10)
        self.isaac_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.isaac_callback,
            10)
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
        self.mask_grey_image = None #masked image
        self.semantic_img = None
        self.bridge = CvBridge()
        self.tiwst = Twist()
        self.rgb_image = None  # Stores the latest rgb image
        self.image_dir = '/home/milos/row-following/ros2_ws/Dataset'  # Directory to save images
        self.image_counter = 1
        self.max_images = 100

    def isaac_callback(self, msg):
        # Save the images when a cmd_vel message is received
        if self.rgb_image is not None and self.mask_grey_image is not None and msg.linear.x <= self.max_images:
            # filename = os.path.join(self.image_dir, f'CilantroOm_{self.image_counter}.png')
            filename = os.path.join(self.image_dir, f'CilantroOm_{int(msg.linear.x)}.png')
            cv2.imwrite(filename, self.rgb_image)
            # filename = os.path.join(self.image_dir, f'MaskedCilantroOm_{self.image_counter}.png')
            filename = os.path.join(self.image_dir, f'MaskedCilantroOm_{int(msg.linear.x)}.png')
            cv2.imwrite(filename, self.mask_grey_image)
            # self.get_logger().info(f'Saved image to {filename}')
            self.image_counter += 1
            cv2.imshow('RGB Image', self.rgb_image)
            cv2.imshow('Masked Image', self.mask_grey_image)
            # combined_image = cv2.hconcat([self.rgb_image, self.mask_grey_image])
            # cv2.imshow('Combined Image - Side by Side', combined_image)
            cv2.waitKey(1)
        else:
            self.get_logger().info('No image available to save or the maximum number of images has been reached!')

    def image_callback(self, msg):
        # Update the latest rgb image
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # cv2.imshow('RGB Image', self.rgb_image)
        # cv2.waitKey(1)

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
        # Update the latest semantic image
        self.semantic_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32SC1')
        condition = (self.semantic_img == 1) # Class 1: white Cilantro Om
        condition2 = (self.semantic_img == 2) # Class 2: black Ground
        condition3 = (self.semantic_img == 0) # Class 0: black Background
        # print(condition)
        self.mask_grey_image = np.zeros(self.semantic_img.shape, dtype=np.uint8)
        self.mask_grey_image[condition] = 255
        # self.mask_grey_image[condition2] = 0
        # self.mask_grey_image[condition3] = 0

        #Create a mask
        # self.mask_grey_image = self.colormap[self.semantic_img]
        # self.mask_grey_image = self.safe_color_mapping()

        # cv2.imshow('Segmentation Display', self.mask_grey_image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()