import rclpy
from rclpy.node import Node
from circle_detector_msgs.msg import CircleArray  #######################
import math

class LeadToCBSSubscriber(Node):

    def __init__(self):
        super().__init__('lead_to_cbs_sub_node')
        self.subscription = self.create_subscription(
            CircleArray,
            'circle_locations',
            self.listener_callback,
            10)
        self.subscription
        # Set the initial robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        # Set the target circle coordinates
        self.target_x = 0.0
        self.target_y = 0.0
        # Set the robot control parameters
        self.linear_speed = 0.1
        self.angular_speed = 0.2

    def listener_callback(self, msg):
        if len(msg.circles) > 0:
            # Get the first detected circle as the target
            target_circle = msg.circles[0]
            self.target_x = target_circle.x
            self.target_y = target_circle.y
            # Calculate the difference between the robot and target positions
            dx = self.target_x - self.robot_x
            dy = self.target_y - self.robot_y
            # Calculate the angle between the robot and target positions
            angle = math.atan2(dy, dx)
            # Calculate the distance between the robot and target positions
            distance = math.sqrt(dx*dx + dy*dy)
            # Adjust the robot orientation to face the target
            if abs(angle) > 0.1:
                # Generate an angular control command
                angular_cmd = self.angular_speed * angle / abs(angle)
                # Send the angular control command to the robot
                # (replace this with your own code to send commands to your robot controller or motor driver)
                send_angular_command(angular_cmd)
            else:
                # Stop the robot rotation
                # (replace this with your own code to send commands to your robot controller or motor driver)
                send_angular_command(0.0)
            # Move the robot towards the target
            if distance > 0.1:
                # Generate a linear control command
                linear_cmd = self.linear_speed * distance / abs(distance)
                # Send the linear control command to the robot
                # (replace this with your own code to send commands to your robot controller or motor driver)
                send_linear_command(linear_cmd)
            else:
                # Stop the robot movement
                # (replace this with your own code to send commands to your robot controller or motor driver)
                send_linear_command(0.0)

    def send_angular_command(self, cmd):
        # Replace this with your own code to send commands to your robot controller or motor driver
        pass

    def send_linear_command(self, cmd):
        # Replace this with your own code to send commands to your robot controller or motor driver
        pass


def main(args=None):
    rclpy.init(args=args)
    lead_to_cbs_sub_node = LeadToCBSSubscriber()
    rclpy.spin(lead_to_cbs_sub_node)
    lead_to_cbs_sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
