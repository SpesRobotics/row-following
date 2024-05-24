#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

import math
import numpy as np
import time

from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory
import os

import pymap3d as pm
from nav2_test import FollowPathClient

package_dir = get_package_share_directory("row_following_bringup")
yolo_model_path = os.path.join(package_dir, 'resource', 'best.pt')

video_path = "/home/pilaciv/Workspaces/row-following/row_following_bringup/resource/wide1.mp4"

classNames = ['Ground', 'CilantroOm']

last_time = 0
integral = 0

previous = 0

kp = 0.5
ki = 0
kd = 0.1

def pid(error):
    global last_time, integral, previous
    now = time.time()
    dt = now - last_time / 1000.0
    last_time = now

    proportional = error
    integral += error * dt
    derivative = (error - previous) / dt
    previous = error

    output = kp * proportional + ki * integral + kd * derivative

    return output

class RowFollow(Node):
    def __init__(self):
        super().__init__('row_follow_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.image_subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            1)
        self.image_subscription2 = self.create_subscription(
            Image,
            '/video/rgb',
            self.video_callback,
            1)
        self.subscription = self.create_subscription(
            NavSatFix,
            '/navsatfix',
            self.navsat_callback,
            10
        )

        self.model = YOLO(yolo_model_path)
        self.mode = 'ht'
        self.out = False
        self.ground_center_coordinates = []

        self.ref_lat = 45.2586161
        self.ref_lon = 19.8066591
        self.ref_alt = 76.0

        self.get_logger().info('Row Following Started!')

    def find_angle_diff(self, image, ground_center_coordinates):
            for coord in ground_center_coordinates:
                cv2.circle(image, coord, 5, (255, 0, 0), -1)
            height, width, channels = image.shape
            center_coordinates = (width // 2, height // 2)

            cv2.circle(image, center_coordinates, 5, (0, 0, 255), -1)

            cv2.line(image, (center_coordinates[0], height), center_coordinates, (0, 0, 255), 3)

            closest_coordinate = min(ground_center_coordinates, key=lambda coord: abs(coord[0] - center_coordinates[0]))

            cv2.line(image, (center_coordinates[0], height), closest_coordinate, (255, 0, 0), 3)

            adjacent = height / 2
            hypotenuse = math.sqrt((height / 2) ** 2 + (abs(center_coordinates[0] - closest_coordinate[0])) ** 2)

            angle_radians = math.acos(adjacent / hypotenuse)

            angle_degrees = math.degrees(angle_radians)

            cv2.putText(image, f'{angle_degrees:.2f}deg', (center_coordinates[0], center_coordinates[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            desired_value = 0.0
            actual_value = angle_radians
            error = abs(desired_value - actual_value)
            output = pid(error)

            if center_coordinates[0] > closest_coordinate[0]:
                return output
            elif center_coordinates[0] < closest_coordinate[0]:
                return -output
            else:
                return 0.0

    def image_callback(self, msg):
        self.bridge = CvBridge()
        cmd_vel = Twist()
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, channels = rgb_image.shape
        cropped_rgb_image = rgb_image[int((height / 2) - 40) : int((height / 2) + 40), :] #Crop the image
        height, width, channels = cropped_rgb_image.shape
        output = 0.0

        # image = cv2.circle(cropped_rgb_image, center_coordinates, 5, (0, 0, 255), -1)

        if self.mode == 'ml':
            results = self.model(cropped_rgb_image)

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    w, h = x2 - x1, y2 - y1

                    conf = math.ceil((box.conf[0] * 100)) / 100

                    cls = int(box.cls[0])

                    class_colors = {
                        0: (255, 0, 0),   # Blue for 'Ground'
                        1: (0, 255, 0)    # Green for 'CilantroOm'
                    }
                    color = class_colors.get(cls, (0, 0, 255))  # Default to red if class not found
                    if cls == 0:
                        self.ground_center_coordinates.append((x1 + (w // 2), y1 + (h // 2)))
                        # output = self.find_angle_diff(cropped_rgb_image, ground_center_coordinates)

                    cv2.rectangle(cropped_rgb_image, (x1, y1), (x1 + w, y1 + h), color, 2)
                    cv2.putText(cropped_rgb_image, f'{classNames[cls]} {conf}', (max(0, x1), max(35, y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                if self.ground_center_coordinates:
                    output = self.find_angle_diff(cropped_rgb_image, self.ground_center_coordinates)
                    self.ground_center_coordinates.clear()

            cv2.imshow("YOLO Output", cropped_rgb_image)
            cv2.waitKey(1)

        elif self.mode == 'ht':
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            blurred_img = cv2.GaussianBlur(gray_image, (305, 305), 0) #105 105 (155 155 sa 100 100)
            edges = cv2.Canny(blurred_img, 1, 900, apertureSize=7) # 5, 6
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=700, maxLineGap=50)
            
            # gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            
            # blurred_img = cv2.GaussianBlur(gray_image, (205, 205), 0) #105 105 (155 155 sa 100 100)
            # adaptive_thresh_mean = cv2.adaptiveThreshold(
            #     blurred_img,
            #     maxValue=255,
            #     adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
            #     thresholdType=cv2.THRESH_BINARY,
            #     blockSize=355,
            #     C=3
            # )
            # inverted_image = cv2.bitwise_not(adaptive_thresh_mean)
            # # edges = cv2.Canny(adaptive_thresh_mean, 700, 700, apertureSize=5) # 5, 6
            # lines = cv2.HoughLinesP(inverted_image, 1, np.pi / 180, 10, minLineLength=500, maxLineGap=90)
            
            line1 = None
            line2 = None
            min_diff1 = 150
            min_diff2 = 150
            # Target x1 values
            target_x1_1 = 340
            target_x1_2 = 820

            # Find the closest lines to the target x1 values
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi

                    if -90 <= angle <= -60:  # Filter mostly vertical lines
                        diff1 = abs(x1 - target_x1_1)
                        if diff1 < min_diff1:
                            min_diff1 = diff1
                            line1 = line[0]

                    if 60 <= angle <= 90:
                        diff2 = abs(x1 - target_x1_2)
                        if diff2 < min_diff2:
                            min_diff2 = diff2
                            line2 = line[0]

            if line1 is not None and line2 is not None:
                x1_1, y1_1, x2_1, y2_1 = line1
                line1 = None
                x1_1 = x1_1 - 90
                x2_1 = x2_1 - 90
                cv2.line(rgb_image, (x1_1, y1_1), (x2_1, y2_1), (255, 0, 0), 2)
                # x1_2, y1_2, x2_2, y2_2 = line2
                x2_2, y2_2, x1_2, y1_2 = line2
                line2 = None
                cv2.line(rgb_image, (x1_2, y1_2), (x2_2, y2_2), (255, 0, 0), 2)

                x1_center = int((x1_1 + x1_2) / 2)
                x2_center = int((x2_1 + x2_2) / 2)
                y1_center = int((y1_1 + y1_2) / 2)
                y2_center = int((y2_1 + y2_2) / 2)
                
                print(y1_center, y1_1, y1_2)
                print(y2_center, y2_1, y2_2)
                cv2.line(rgb_image, (x1_center, y1_center), (x2_center, y2_center), (255, 0, 0), 2)
                
                self.ground_center_coordinates.append(((x1_center + x2_center) // 2, (y1_center + y2_center) // 2))

            if self.ground_center_coordinates:
                output = self.find_angle_diff(rgb_image, self.ground_center_coordinates)
                self.ground_center_coordinates.clear()


            # Display the image
            cv2.imshow('Detected Lines', rgb_image)
            cv2.waitKey(1)

        if self.mode == 'turn':
            return

        cmd_vel.linear.x = 0.1
        cmd_vel.angular.z = output
        self.publisher_.publish(cmd_vel)

    def video_callback(self, msg):
        self.bridge = CvBridge()
        cmd_vel = Twist()
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, channels = rgb_image.shape
        cropped_rgb_image = rgb_image[int((height / 2) - 40) : int((height / 2) + 40), :] #Crop the image
        height, width, channels = cropped_rgb_image.shape
        output = 0.0

        if self.mode == 'ml':
            results = self.model(cropped_rgb_image)

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    w, h = x2 - x1, y2 - y1

                    conf = math.ceil((box.conf[0] * 100)) / 100

                    cls = int(box.cls[0])

                    class_colors = {
                        0: (255, 0, 0),   # Blue for 'Ground'
                        1: (0, 255, 0)    # Green for 'CilantroOm'
                    }
                    color = class_colors.get(cls, (0, 0, 255))  # Default to red if class not found
                    if cls == 0:
                        self.ground_center_coordinates.append((x1 + (w // 2), y1 + (h // 2)))

                    cv2.rectangle(cropped_rgb_image, (x1, y1), (x1 + w, y1 + h), color, 2)
                    cv2.putText(cropped_rgb_image, f'{classNames[cls]} {conf}', (max(0, x1), max(35, y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                if self.ground_center_coordinates:
                    output = self.find_angle_diff(cropped_rgb_image, self.ground_center_coordinates)
                    self.ground_center_coordinates.clear()

            cv2.imshow("YOLO Output", cropped_rgb_image)
            cv2.waitKey(1)

        elif self.mode == 'ht':
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            blurred_img = cv2.GaussianBlur(gray_image, (225, 225), 0) #105 105 (155 155 sa 100 100)
            adaptive_thresh_mean = cv2.adaptiveThreshold(
                blurred_img,
                maxValue=255,
                adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
                thresholdType=cv2.THRESH_BINARY,
                blockSize=335,
                C=3
            )
            inverted_image = cv2.bitwise_not(adaptive_thresh_mean)
            # edges = cv2.Canny(adaptive_thresh_mean, 700, 700, apertureSize=5) # 5, 6
            lines = cv2.HoughLinesP(inverted_image, 1, np.pi / 180, 10, minLineLength=500, maxLineGap=90)
            # blurred_img = cv2.GaussianBlur(gray_image, (199, 195), 0) #105 105 (155 155 sa 100 100)
            # adaptive_thresh_mean = cv2.adaptiveThreshold(
            #     blurred_img,
            #     maxValue=255,
            #     adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
            #     thresholdType=cv2.THRESH_BINARY,
            #     blockSize=255,
            #     C=3
            # )
            # edges = cv2.Canny(adaptive_thresh_mean, 700, 700, apertureSize=5) # 5, 6
            # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 10, minLineLength=400, maxLineGap=300)

            # blurred_img = cv2.GaussianBlur(gray_image, (105, 105), 0) #105 105 (155 155 sa 100 100)
            # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 17))
            # closed_image = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
            # lines = cv2.HoughLinesP(closed_image, 1, np.pi / 180, 100, minLineLength=200, maxLineGap=20)

            # blurred_img = cv2.GaussianBlur(gray_image, (105, 105), 0) #105 105 (155 155 sa 100 100)
            # edges = cv2.Canny(blurred_img, 70, 120, apertureSize=5) # 5, 6
            # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 17))
            # closed_image = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
            # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=0, maxL
            line1 = None
            line2 = None
            min_diff1 = 150
            min_diff2 = 150
            # Target x1 values
            target_x1_1 = 380
            target_x1_2 = 820

            # Find the closest lines to the target x1 values
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                    # print(angle)
                    if -90 <= angle <= -60:  # Filter mostly vertical lines
                        diff1 = abs(x1 - target_x1_1)
                        if diff1 < min_diff1:
                            min_diff1 = diff1
                            line1 = line[0]

                    if 60 <= angle <= 90:
                        diff2 = abs(x1 - target_x1_2)
                        if diff2 < min_diff2:
                            min_diff2 = diff2
                            line2 = line[0]

            if line1 is not None and line2 is not None:
                x1_1, y1_1, x2_1, y2_1 = line1
                line1 = None
                x1_1 = x1_1 - 90
                x2_1 = x2_1 - 90
                cv2.line(rgb_image, (x1_1, y1_1), (x2_1, y2_1), (255, 0, 0), 2)
                # x1_2, y1_2, x2_2, y2_2 = line2
                x2_2, y2_2, x1_2, y1_2 = line2
                line2 = None
                print(line1)
                print(line2)
                cv2.line(rgb_image, (x1_2, y1_2), (x2_2, y2_2), (255, 0, 0), 2)

                x1_center = int((x1_1 + x1_2) / 2)
                x2_center = int((x2_1 + x2_2) / 2)
                y1_center = int((y1_1 + y1_2) / 2)
                y2_center = int((y2_1 + y2_2) / 2)
                
                print(y1_center, y1_1, y1_2)
                print(y2_center, y2_1, y2_2)
                cv2.line(rgb_image, (x1_center, y1_center), (x2_center, y2_center), (255, 0, 0), 2)
                self.ground_center_coordinates.append(((x1_center + x2_center) // 2, (y1_center + y2_center) // 2))

            if self.ground_center_coordinates:
                output = self.find_angle_diff(rgb_image, self.ground_center_coordinates)
                self.ground_center_coordinates.clear()

            # Display the image
            cv2.imshow('Detected Lines', rgb_image)
            cv2.waitKey(1)

        if self.mode == 'turn':
            return

        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = output
        self.publisher_.publish(cmd_vel)

    def navsat_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        x, y, z = pm.geodetic2enu(lat=lat, lon=lon, h=alt,lat0=self.ref_lat, lon0=self.ref_lon, h0=self.ref_alt)
        if x >= 3.5 and self.out == False:
            self.out = True
            self.mode = 'turn'
            obj = FollowPathClient()
            while rclpy.ok() and obj.flag:
                rclpy.spin_once(obj, timeout_sec=0.1)
            self.get_logger().info('Path goal reached or aborted.')
            self.get_logger().info('End of the row!')
            self.mode = 'ml'
        else:
            self.mode = 'ht'    

def main(args=None):
    rclpy.init(args=args)
    row_follow_node = RowFollow()
    while rclpy.ok():
        rclpy.spin(row_follow_node)
        # rclpy.spin_once(row_follow_node)
    row_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()