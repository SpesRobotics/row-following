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
            '/rgb',
            self.image_callback,
            1)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            return
        
        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Detected Lines', edges)

        # # Create kernel
        # kernel_size = (30, 30)  # Adjust size based on how strong you want the dilation and erosion to be
        # kernel = np.ones(kernel_size, np.uint8)

        # # Apply dilation
        # dilated_image = cv2.dilate(gray_image, kernel, iterations=3)

        # # Apply erosion
        # erosion_image = cv2.erode(dilated_image, kernel, iterations=3)

        # Detect edges using Canny
        edges = cv2.Canny(gray_image, 50, 150, apertureSize=3)
        cv2.imshow('Detected Lines', edges)

        # Perform Hough Line transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Display the image
        # cv2.imshow('Detected Lines', cv_image)
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
