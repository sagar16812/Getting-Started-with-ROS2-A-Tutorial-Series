import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from example_interfaces.action import MoveTurtle
from rclpy.duration import Duration

class TurtleActionServer(Node):
    def __init__(self):
        super().__init__('turtle_action_server')
        self._action_server = ActionServer(
            self,
            MoveTurtle,
            'move_turtle',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.current_pose = None

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def update_pose(self, msg):
        self.current_pose = msg

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = MoveTurtle.Feedback()
        twist_msg = Twist()
        
        twist_msg.linear.x = 1.0  # Move forward with linear velocity of 1.0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                return MoveTurtle.Result()
            
            self.publisher.publish(twist_msg)
            if self.current_pose:
                feedback_msg.current_position = self.current_pose.x
                goal_handle.publish_feedback(feedback_msg)

                if self.current_pose.x >= goal_handle.request.distance:
                    self.get_logger().info('Goal reached')
                    twist_msg.linear.x = 0.0
                    self.publisher.publish(twist_msg)
                    goal_handle.succeed()
                    return MoveTurtle.Result()

            await goal_handle.get_goal_service().publish_feedback(feedback_msg)
            await self.sleep(Duration(seconds=0.1))

def main(args=None):
    rclpy.init(args=args)
    turtle_action_server = TurtleActionServer()
    rclpy.spin(turtle_action_server)
    turtle_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
