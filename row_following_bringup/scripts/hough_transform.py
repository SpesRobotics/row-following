import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            '/video/rgb',
            self.image_callback,
            1)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            return

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # blurred_img = cv2.GaussianBlur(gray_image, (95, 95), 0) #105 105 (155 155 sa 100 100)
        # edges = cv2.Canny(blurred_img, 1500, 1500, apertureSize=7) # 5, 6
        # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=0, maxLineGap=60)


        # blurred_img = cv2.GaussianBlur(gray_image, (105, 105), 0) #105 105 (155 155 sa 100 100)
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 17))
        # closed_image = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        # lines = cv2.HoughLinesP(closed_image, 1, np.pi / 180, 100, minLineLength=200, maxLineGap=20)

        blurred_img = cv2.GaussianBlur(gray_image, (105, 105), 0) #105 105 (155 155 sa 100 100)
        edges = cv2.Canny(blurred_img, 70, 120, apertureSize=5) # 5, 6
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 17))
        closed_image = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=0, maxLineGap=100)



        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                if 60 <= np.abs(angle) <= 90:  # Filter mostly vertical lines
                    cv2.line(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # Display the image
        cv2.imshow('Detected Lines', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    # Clean up OpenCV
    cv2.destroyAllWindows()
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()