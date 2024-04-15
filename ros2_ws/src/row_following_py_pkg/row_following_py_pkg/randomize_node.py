import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 1  # seconds (publish every 1 second)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.msg = Twist()  # Initialize the Twist message here

    def timer_callback(self):
        change = 0.1
        # Increment each component of the message
        self.msg.linear.x += change
        self.msg.linear.y += change
        self.msg.linear.z += change
        self.msg.angular.x += change
        self.msg.angular.y += change
        self.msg.angular.z += change
        self.publisher_.publish(self.msg)
        # Log the current state of the message
        self.get_logger().info('Publishing: "%s"' % self.msg)

def main(args=None):
    rclpy.init(args=args)
    twist_publisher = TwistPublisher()
    rclpy.spin(twist_publisher)
    twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
