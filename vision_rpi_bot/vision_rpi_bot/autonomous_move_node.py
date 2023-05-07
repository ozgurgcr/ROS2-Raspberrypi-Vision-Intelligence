import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from circle_detector_msgs.msg import CircleArray


class AutonomousMoveNode(Node):
    def __init__(self):
        super().__init__('autonomous_move_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            CircleArray,
            'circle_locations',
            self.circle_callback,
            10)
        self.current_x_ = None
        self.target_x_ = None

    def circle_callback(self, msg):
        if len(msg.circles) == 0:
            # If no circle is detected, keep moving forward
            self.move_forward()
        else:
            # If a circle is detected, stop moving and turn towards the circle
            self.stop_moving()
            circle = msg.circles[0]
            self.current_x_ = circle.x
            self.target_x_ = 0
            self.turn_towards_target()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def stop_moving(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def turn_towards_target(self):
        twist = Twist()
        # Use a proportional controller to adjust the angular velocity
        k_p = 0.01
        twist.angular.z = -k_p * (self.current_x_ - self.target_x_)
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    autonomous_move_node = AutonomousMoveNode()
    rclpy.spin(autonomous_move_node)
    autonomous_move_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
