import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from circle_detector_msgs.msg import CircleArray, Circle    # this going to be change

#################################### the code will be configured for CBs ####################

class ImageProcessorPub(Node):
    def __init__(self):
        super().__init__('image_processor_pub')
        self.publisher_ = self.create_publisher(CircleArray, 'circle_locations', 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):                          # this is going to be change
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        circles = self.detect_circles(cv_image)
        self.publish_circles(circles)

    def detect_circles(self, img):                                 # this is going to be change 
        # TODO: Replace with your camera intrinsics and extrinsics parameters
        fx = 1
        fy = 1
        cx = img.shape[1] / 2
        cy = img.shape[0] / 2
        camera_position = np.array([0, 0, 0])
        camera_orientation = np.eye(3)

        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply HoughCircles to detect circles
        circles = cv2.HoughCircles(gray_blur, cv2.HOUGH_GRADIENT, 1, minDist=int(min(gray_blur.shape[0], gray_blur.shape[1]) / 8), param1=100, param2=30, minRadius=10, maxRadius=50)

        # Convert circles to 3D world coordinates
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Compute the 3D coordinates of the circle center
                X = (x - cx) / fx
                Y = (y - cy) / fy
                Z = 1
                circle_pos = np.dot(camera_orientation.T, np.array([X, Y, Z]))
                circle_pos = circle_pos / np.linalg.norm(circle_pos) * r + camera_position
                # Update the circle's x and y coordinates with the 3D coordinates
                x, y, _ = circle_pos
                yield x, y

    def publish_circles(self, circles):
        msg = CircleArray()
        for x, y in circles:
            circle_msg = Circle()
            circle_msg.x = x
            circle_msg.y = y
            msg.circles.append(circle_msg)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    image_processor_pub = ImageProcessorPub()
    rclpy.spin(image_processor_pub)
    image_processor_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
