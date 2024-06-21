#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import math
import numpy as np
import time
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os

from rclpy.executors import MultiThreadedExecutor

package_dir = get_package_share_directory("row_following_bringup")
yolo_model_path = os.path.join(package_dir, "resource", "best.pt")

classNames = ["Ground", "CilantroOm"]

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
        super().__init__("row_follow_node")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 1)
        self.camera_publisher = self.create_publisher(Image, "camera/robot/view", 10)
        self.image_subscription = self.create_subscription(
            Image, "/rgb", self.image_callback, 1
        )
        self.image_subscription = self.create_subscription(
            String, "/out/of/field", self.out_of_field_callback, 1
        )

        self.model = YOLO(yolo_model_path)
        self.mode = None
        self.last_msg_time = time.time()
        self.timer = self.create_timer(0.1, self.check_message_timeout)

        self.ref_lat = 45.2586161
        self.ref_lon = 19.8066591
        self.ref_alt = 76.0

        self.get_logger().info("Row Following Started!")

    def check_message_timeout(self):
        current_time = time.time()
        if current_time - self.last_msg_time > 0.5:
            self.mode = None
            print("No message received for 0.5 seconds - assuming out of the field!")

    def out_of_field_callback(self, msg):
        self.last_msg_time = time.time()

        if msg:
            self.mode = msg.data
        else:
            self.mode = None

    def find_angle_diff(self, image, ground_center_coordinates):
        for coord in ground_center_coordinates:
            cv2.circle(image, coord, 5, (255, 0, 0), -1)
        height, width, channels = image.shape
        center_coordinates = (width // 2, height // 2)

        cv2.circle(image, center_coordinates, 5, (0, 0, 255), -1)

        cv2.line(
            image, (center_coordinates[0], height), center_coordinates, (0, 0, 255), 3
        )

        closest_coordinate = min(
            ground_center_coordinates,
            key=lambda coord: abs(coord[0] - center_coordinates[0]),
        )

        cv2.line(
            image, (center_coordinates[0], height), closest_coordinate, (255, 0, 0), 3
        )

        adjacent = height / 2
        hypotenuse = math.sqrt(
            (height / 2) ** 2
            + (abs(center_coordinates[0] - closest_coordinate[0])) ** 2
        )

        angle_radians = math.acos(adjacent / hypotenuse)

        angle_degrees = math.degrees(angle_radians)

        cv2.putText(
            image,
            f"{angle_degrees:.2f}deg",
            (center_coordinates[0], center_coordinates[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )

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
        if self.mode is None:
            return

        self.bridge = CvBridge()
        cmd_vel = Twist()
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, channels = rgb_image.shape
        cropped_rgb_image = rgb_image[
            int((height / 2) - 40) : int((height / 2) + 40), :
        ]
        height, width, channels = cropped_rgb_image.shape
        output = 0.0
        self.ground_center_coordinates = []

        if self.mode == "ml":
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
                        0: (255, 0, 0),  # Blue for 'Ground'
                        1: (0, 255, 0),  # Green for 'CilantroOm'
                    }
                    color = class_colors.get(cls, (0, 0, 255))
                    if cls == 0:
                        self.ground_center_coordinates.append(
                            (x1 + (w // 2), y1 + (h // 2))
                        )

                    cv2.rectangle(
                        cropped_rgb_image, (x1, y1), (x1 + w, y1 + h), color, 2
                    )
                    cv2.putText(
                        cropped_rgb_image,
                        f"{classNames[cls]} {conf}",
                        (max(0, x1), max(35, y1)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2,
                    )

                if self.ground_center_coordinates:
                    output = self.find_angle_diff(
                        cropped_rgb_image, self.ground_center_coordinates
                    )
                    self.ground_center_coordinates.clear()

            processed_image_msg = self.bridge.cv2_to_imgmsg(
                cropped_rgb_image, encoding="bgr8"
            )
            self.camera_publisher.publish(processed_image_msg)

        elif self.mode == "ht":
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

            blurred_img = cv2.GaussianBlur(gray_image, (205, 205), 0)
            adaptive_thresh_mean = cv2.adaptiveThreshold(
                blurred_img,
                maxValue=255,
                adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
                thresholdType=cv2.THRESH_BINARY,
                blockSize=355,
                C=3,
            )
            inverted_image = cv2.bitwise_not(adaptive_thresh_mean)
            lines = cv2.HoughLinesP(
                inverted_image, 1, np.pi / 180, 10, minLineLength=500, maxLineGap=90
            )

            line1 = None
            line2 = None
            min_diff1 = 150
            min_diff2 = 150
            target_x1_1 = 340
            target_x1_2 = 820

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi

                    if -90 <= angle <= -60:
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
                x2_2, y2_2, x1_2, y1_2 = line2
                line2 = None
                cv2.line(rgb_image, (x1_2, y1_2), (x2_2, y2_2), (255, 0, 0), 2)

                x1_center = int((x1_1 + x1_2) / 2)
                x2_center = int((x2_1 + x2_2) / 2)
                y1_center = int((y1_1 + y1_2) / 2)
                y2_center = int((y2_1 + y2_2) / 2)

                cv2.line(
                    rgb_image,
                    (x1_center, y1_center),
                    (x2_center, y2_center),
                    (255, 0, 0),
                    2,
                )

                self.ground_center_coordinates.append(
                    ((x1_center + x2_center) // 2, (y1_center + y2_center) // 2)
                )

            if self.ground_center_coordinates:
                output = self.find_angle_diff(rgb_image, self.ground_center_coordinates)
                self.ground_center_coordinates.clear()

            processed_image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
            self.camera_publisher.publish(processed_image_msg)

        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = output
        self.publisher_.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    row_follow_node = RowFollow()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(row_follow_node)
    try:
        executor.spin()
    finally:
        row_follow_node.destroy_node()
        rclpy.shutdown()

    rclpy.spin(row_follow_node)
    row_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
