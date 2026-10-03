import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Detector(Node):
    def __init__(self):
        super().__init__("detector")
        self.br = CvBridge()
        self.create_subscription(Image, "/writer/image", self.cb, 10)
        self.pub = self.create_publisher(Image, "/writer/image_up", 10)

    def cb(self, m):
        img = cv2.flip(self.br.imgmsg_to_cv2(m, "bgr8"), -1)
        self.pub.publish(self.br.cv2_to_imgmsg(img, "bgr8"))

rclpy.init()
rclpy.spin(Detector())
