#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

import numpy as np
import math

import os
import time


ssh = True
if ssh:
    directory_path = '/home/milos/row-following/ml/Images' # Path to save the dataset
else:
    directory_path = '/home/pilaciv/Workspaces/row-following/ml/Images' # Path to save the dataset


classNames = ['Ground', 'CilantroOm']

def convert_bbox_to_yolo_format(x, y, w, h, image_width, image_height):
    x_center = (x + w / 2) / image_width
    y_center = (y + h / 2) / image_height
    w_normalized = w / image_width
    h_normalized = h / image_height
    return [x_center, y_center, w_normalized, h_normalized]

def draw_bounding_boxes(gray_image, min_area, fname_index):
    # Get image dimensions
    height, width = gray_image.shape  # This will correctly unpack two values since gray_image is guaranteed to be grayscale

    # Convert to a binary image and invert it (if needed based on your image colors)
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY_INV)

    # Create kernel
    kernel_size = (30, 30)  # Adjust size based on how strong you want the dilation and erosion to be
    kernel = np.ones(kernel_size, np.uint8)

    # Apply dilation
    dilated_image = cv2.dilate(binary_image, kernel, iterations=3)

    # Apply erosion
    erosion_image = cv2.erode(dilated_image, kernel, iterations=3)

    # Optionally apply closing to ensure holes within the dilated areas are filled
    # closed_image = cv2.morphologyEx(dilated_image, cv2.MORPH_CLOSE, kernel)

    # Find all contours
    contours, _ = cv2.findContours(erosion_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours by area, keeping only those above a certain size
    filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_area]

    # Create a color image for visualization
    color_image = cv2.cvtColor(erosion_image, cv2.COLOR_GRAY2BGR)

    # Sort filtered contours by area, descending
    filtered_contours = sorted(filtered_contours, key=cv2.contourArea, reverse=True)

    # Prepare to store the bounding boxes for later use
    green_bounding_boxes = []
    yolo_labels = []

    # Check if there are at least two contours, else take as many as available
    max_contours = min(3, len(filtered_contours))
    
    for i in range(max_contours):
        contour = filtered_contours[i]
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw bounding box in green
        green_bounding_boxes.append((x, y, x+w, y+h))
        # Convert the bounding box to YOLO format
        yolo_bbox = convert_bbox_to_yolo_format(x, y, w, h, width, height)
        
        # Prepare the YOLO format line and append to list (assuming class index 0)
        yolo_labels.append([1] + yolo_bbox)

    green_bounding_boxes.sort(key=lambda box: box[0])  # Sort the rectangles from left to right
    # Finding gaps between bounding boxes
    for i in range(len(green_bounding_boxes) - 1):
        right_of_first_box = green_bounding_boxes[i][2]
        left_of_second_box = green_bounding_boxes[i + 1][0]
        y_top = green_bounding_boxes[i][1]
        y_bottom = green_bounding_boxes[i][3]
        gap_w = left_of_second_box - right_of_first_box
        gap_h = y_bottom - y_top

        # Check if there is a gap
        if left_of_second_box > right_of_first_box:
            # Draw rectangle in the gap
            cv2.rectangle(color_image, (right_of_first_box, green_bounding_boxes[i][1]), (left_of_second_box, green_bounding_boxes[i][3]), (255, 0, 0), 2)  # Blue rectangles
            yolo_bbox = convert_bbox_to_yolo_format(right_of_first_box, y_top, gap_w, gap_h, width, height)
            yolo_labels.append([0] + yolo_bbox)  # Append list of class index and YOLO bbox coordinates

    # Handling edge cases, drawing rectangles from image edges to the first/last box if needed
    if green_bounding_boxes:
        first_box = green_bounding_boxes[0]
        last_box = green_bounding_boxes[-1] #Last item in the list
        # Draw from image start to first box
        if first_box[0] > 0:
            x, y, w, h = 0, first_box[1], first_box[0], first_box[3] - first_box[1]
            cv2.rectangle(color_image, (0, first_box[1]), (first_box[0], first_box[3]), (255, 0, 0), 2)
            yolo_bbox = convert_bbox_to_yolo_format(x, y, w, h, width, height)
            yolo_labels.append([0] + yolo_bbox)  # Assuming class index 0 for blue boxes

        if last_box[2] < gray_image.shape[1]: # Draw from last box to image end
            x, y, w, h = last_box[2], last_box[1], width - last_box[2], last_box[3] - last_box[1]
            cv2.rectangle(color_image, (last_box[2], last_box[1]), (gray_image.shape[1], last_box[3]), (255, 0, 0), 2)
            yolo_bbox = convert_bbox_to_yolo_format(x, y, w, h, width, height)
            yolo_labels.append([0] + yolo_bbox)
            
    # If there is just ground on the picture      
    else:
            x, y, w, h = 0, 0, width, height
            cv2.rectangle(color_image, (0, 0), (width, height), (255, 0, 0), 2)
            yolo_bbox = convert_bbox_to_yolo_format(x, y, w, h, width, height)
            yolo_labels.append([0] + yolo_bbox)

    filename = f"{directory_path}/CilantroOm_{fname_index}.txt"

    with open(filename, "w") as file:
        for label in yolo_labels:
            file.write(" ".join(map(str, label)) + "\n")

    return color_image

class DatasetGenerator(Node):
    def __init__(self):
        super().__init__('dataset_generator')
        self.publisher_ = self.create_publisher(Twist, 'isaac/dataset', 1)
        self.image_subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            1)
        self.subscription = self.create_subscription(
            Image,
            '/semantic_segmentation',
            self.segmentation_callback,
            1)
        self.msg = Twist()
        self.bridge = CvBridge()
        self.save_success = False
        self.save_success2 = False
        self.image_counter = 1
        self.max_images = 300
        self.run_node = True  # For executing the program
        self.get_logger().info('Collecting Simulation Dataset!')

    def publish_message(self):
        change = 1
        if self.save_success and self.save_success2:
            self.msg.linear.x += change # Stores the number of image
            self.msg.linear.y += change
            self.msg.linear.z += change
            self.msg.angular.x += change
            self.msg.angular.y += change
            self.msg.angular.z += change
            self.publisher_.publish(self.msg)
            time.sleep(1) #0.2 Wait for the camera in IsaacSim to get to the posititon
            self.save_success = False
            self.save_success2 = False

    def image_callback(self, msg):
        if int(self.msg.linear.x) < self.max_images:
            # Update the latest rgb image
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(directory_path, f"CilantroOm_{int(self.msg.linear.x)}.png")
            
            height, width, channels = rgb_image.shape

            cropped_rgb_image = rgb_image[int((height / 2) - 40) : int((height / 2) + 40), :] #Crop the image

            self.save_success = cv2.imwrite(filename, cropped_rgb_image)
            self.publish_message()
        elif int(self.msg.linear.x) >= self.max_images:
            self.get_logger().info('Simulation Dataset Collected!')
            self.run_node = False

        # results = self.model(cropped_rgb_image)

        # for r in results:
        #     boxes = r.boxes
        #     for box in boxes:
        #         x1, y1, x2, y2 = box.xyxy[0]
        #         x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

        #         w, h = x2 - x1, y2 - y1

        #         conf = math.ceil((box.conf[0] * 100)) / 100

        #         cls = int(box.cls[0])

        #         class_colors = {
        #             0: (255, 0, 0),   # Blue for 'CilantroOm'
        #             1: (0, 255, 0)    # Green for 'Ground'
        #         }
        #         color = class_colors.get(cls, (0, 0, 255))  # Default to red if class not found

        #         cv2.rectangle(cropped_rgb_image, (x1, y1), (x1 + w, y1 + h), color, 2)
        #         cv2.putText(cropped_rgb_image, f'{classNames[cls]} {conf}', (max(0, x1), max(35, y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)




        # cv2.imshow("YOLO Output", cropped_rgb_image)
        # cv2.waitKey(1)

    def segmentation_callback(self, msg):
        if int(self.msg.linear.x) < self.max_images:
            # Update the latest semantic image
            semantic_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32SC1')
            condition = (semantic_img == 1) # Class 1: white Cilantro Om
            condition2 = (semantic_img == 2) # Class 2: black Ground
            condition3 = (semantic_img == 0) # Class 0: black Background
            # print(condition)
            mask_grey_image = np.zeros(semantic_img.shape, dtype=np.uint8)
            mask_grey_image[condition2] = 255
            filename = os.path.join(directory_path, f"MaskedCilantroOm_{int(self.msg.linear.x)}.png")

            height, width = mask_grey_image.shape

            # cropped_masked_image  = mask_grey_image[int((height / 2) - 40) : int((height / 2) + 40), :] # Crop the image

            bounding_box_img = draw_bounding_boxes(mask_grey_image, 0, int(self.msg.linear.x)) # Draw bounding boxes

            self.save_success2 = cv2.imwrite(filename, bounding_box_img)
            self.publish_message()
        elif int(self.msg.linear.x) >= self.max_images:
            self.get_logger().info('Simulation Dataset Collected!')
            self.run_node = False

def main(args=None):
    rclpy.init(args=args)
    main_node = DatasetGenerator()
    # rclpy.spin(main_node)
    while rclpy.ok() and main_node.run_node:
        rclpy.spin_once(main_node)
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
