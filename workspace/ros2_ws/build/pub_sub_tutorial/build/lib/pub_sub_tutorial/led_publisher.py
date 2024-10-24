import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LedPublisher(Node):
    def __init__(self):
        super().__init__('led_publisher')
        self.publisher_ = self.create_publisher(String, 'led_control', 10)
        self.timer = self.create_timer(2.0, self.publish_command)  # Timer to call publish_command every 2 seconds
        self.command = "turn on"  # Initial command

    def publish_command(self):
        msg = String()
        msg.data = self.command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Toggle between "turn on" and "turn off"
        self.command = "turn on" if self.command == "turn off" else "turn off"

def main(args=None):
    rclpy.init(args=args)
    node = LedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
