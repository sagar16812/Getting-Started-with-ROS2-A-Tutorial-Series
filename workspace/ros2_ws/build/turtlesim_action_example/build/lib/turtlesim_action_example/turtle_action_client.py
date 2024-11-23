import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import MoveTurtle

class TurtleActionClient(Node):
    def __init__(self):
        super().__init__('turtle_action_client')
        self._action_client = ActionClient(self, MoveTurtle, 'move_turtle')

    def send_goal(self, distance):
        goal_msg = MoveTurtle.Goal()
        goal_msg.distance = distance
        self._action_client.wait_for_server()

        # Send the goal and get the future for tracking
        self.goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected.')
            return
        self.get_logger().info('Goal accepted.')
        self.goal_handle = goal_handle

        # Monitor user input for stopping
        while not self.goal_handle.done():
            user_input = input("Type 'stop' to cancel the movement: ").strip().lower()
            if user_input == 'stop':
                self.get_logger().info("Canceling the goal...")
                self.goal_handle.cancel_goal_async()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: Current position on x-axis: {feedback_msg.current_position}')

def main(args=None):
    rclpy.init(args=args)
    action_client = TurtleActionClient()

    # Request distance from user input
    try:
        distance = float(input("Enter the distance for the turtle to move forward: "))
    except ValueError:
        print("Invalid input. Please enter a numeric value.")
        return

    action_client.send_goal(distance)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
