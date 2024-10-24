import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LedSubscriber(Node):
    def __init__(self):
        super().__init__('led_subscriber')
        self.subscription = self.create_subscription(String, 'led_control', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Subscribed: "led_control" - LED has been {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = LedSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
