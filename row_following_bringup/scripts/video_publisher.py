import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/video/rgb', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        video_path = 'row_following_bringup/resource/wide1.mp4'  # Update this to the path of your video file
        self.cap = cv2.VideoCapture(video_path)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        desired_width=1280
        desired_height=720
        # Resize the frame
        resized_frame = cv2.resize(frame, (desired_width, desired_height))

        # Crop the frame
        height, width, channels = resized_frame.shape
        cropped_rgb_image = resized_frame[int((height / 2) - 40): int((height / 2) + 40), :]
        if ret:
            msg = self.bridge.cv2_to_imgmsg(resized_frame, 'bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().warn('Failed to read frame from video. Stopping the timer.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
