import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class BatteryClient(Node):
    def __init__(self):
        super().__init__('battery_client')
        self.cli = self.create_client(Trigger, 'check_battery')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Battery Service...')
        self.request = Trigger.Request()

    def send_request(self):
        future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result().message}')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    client = BatteryClient()
    client.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
