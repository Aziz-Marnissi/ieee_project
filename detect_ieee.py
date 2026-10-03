import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

TOPIC = "/writer/image"

class Det(Node):
    def __init__(self):
        super().__init__("detect_ieee")
        self.br = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.create_subscription(Image, TOPIC, self.cb, qos_profile_sensor_data)

    def cb(self, m):
        frame = self.br.imgmsg_to_cv2(m, "bgr8")
        for r in self.model(frame, classes=[0], conf=0.4, verbose=False):
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"person {float(b.conf[0]):.2f}", (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 245), (180, 25, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) > 400:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, "fire", (x, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("detect_ieee", frame)
        cv2.waitKey(1)

rclpy.init()
node = Det()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
cv2.destroyAllWindows()
node.destroy_node()
rclpy.shutdown()
