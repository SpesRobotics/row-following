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
        self.mask_grey_image[condition2] = 255

        _, binary = cv2.threshold(self.mask_grey_image, 127, 255, cv2.THRESH_BINARY)
        image_with_boxes = cv2.cvtColor(self.mask_grey_image, cv2.COLOR_GRAY2BGR)

        # Find contours from the binary image
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create a copy of the mask to draw bounding boxes on (optional)
        # image_with_boxes = np.copy(self.mask_grey_image)

        # Loop through all detected contours
        for contour in contours:
            # Get the bounding rectangle for each contour
            x, y, w, h = cv2.boundingRect(contour)
            
            # Draw the rectangle
            cv2.rectangle(image_with_boxes, (x, y), (x+w, y+h), (255, 0, 0), 2)  # Draw with blue borders

        # cv2.imshow('Image with Bounding Boxes', image_with_boxes)
        # cv2.waitKey(1)
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

#color_segmentation = self.colormap[self.semantic_img]

# Resulting Array: The result is a new array (color_segmentation) where each pixel in semantic_img has been replaced by the corresponding RGB color from self.colormap. The shape of color_segmentation will be identical to semantic_img, but with an extra dimension for color:

# arduino
# Copy code
# Shape: (height, width, 3)
# Visual Example
# If semantic_img is:

# lua
# Copy code
# [[0, 1],
#  [2, 2]]
# And self.colormap is:

# lua
# Copy code
# [[0, 0, 0],
#  [128, 0, 0],
#  [0, 128, 0]]
# Then self.colormap[semantic_img] results in:

# lua
# Copy code
# [[[  0,   0,   0],   # Black
#   [128,   0,   0]],  # Red

#  [[  0, 128,   0],   # Green
#   [  0, 128,   0]]]  # Green