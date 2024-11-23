import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtlesim.action import RotateAbsolute


class TurtleRotateClient(Node):

    def __init__(self):
        super().__init__('turtle_rotate_client')
        self._action_client = ActionClient(self, RotateAbsolute, '/turtle1/rotate_absolute')

    def send_goal(self, theta):
        # Wait until the action server is available
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = theta

        # Send the goal
        self.get_logger().info(f'Sending goal to rotate turtle to {theta} radians.')
        self._goal_future = self._action_client.send_goal_async(goal_msg)
        self._goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal completed with result: {result}')
        self.ask_for_new_goal()

    def ask_for_new_goal(self):
        theta = float(input("Enter the desired rotation angle in radians: "))
        self.send_goal(theta)


def main(args=None):
    rclpy.init(args=args)

    action_client = TurtleRotateClient()

    # Start by asking for the first rotation angle
    action_client.ask_for_new_goal()

    # Spin the client to process events
    rclpy.spin(action_client)

    # Clean up
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
