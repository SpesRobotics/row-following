import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO

class ImageSubscriber:
    def __init__(self):
        self.node = rclpy.create_node('image_subscriber')
        self.subscription = self.node.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10)
        self.cv_bridge = CvBridge()

        # Load your YOLO model
        self.model = YOLO('/home/milos/row-following/ros2_ws/Dataset/best.pt')

    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform inference using your YOLO model
        results = self.model(cv_image)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                w, h = x2 - x1, y2 - y1
                cv2.rectangle(cv_image, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
                # cv2.putText(cv_image, f"{r.names[r.classes[box.class_id]]}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # print(type(results))  # Check the type of the results
        # print(results)  # This will give you an idea of how the data is structured
        # boxes = results.boxes  # Accessing bounding boxes

        # # Draw bounding boxes on the image
        # for box in boxes.xyxy[0]:  # Assuming boxes.xyxy[0] contains detections for the first image
        #         x_min, y_min, x_max, y_max, conf, cls_id = int(box[0]), int(box[1]), int(box[2]), int(box[3]), box[4], int(box[5])
        #         # Draw rectangle to visualize bounding box
        #         cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        #         # Put class label with confidence score
        #         label = f"{results.names[cls_id]} {conf:.2f}"
        #         cv2.putText(cv_image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image with bounding boxes
        cv2.imshow("YOLO Output", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    subscriber = ImageSubscriber()
    rclpy.spin(subscriber.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
