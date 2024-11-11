import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Using a basic service type for simplicity

class BatteryService(Node):
    def __init__(self):
        super().__init__('battery_service')
        self.srv = self.create_service(Trigger, 'check_battery', self.battery_callback)
        self.get_logger().info('Battery Service Node Started')

    def battery_callback(self, request, response):
        response.success = True
        response.message = "Battery level: 80%"  # Constant value for demonstration
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BatteryService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
