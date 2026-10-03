import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

TOPIC = "/writer/image"
GREEN, ORANGE, RED = (0, 255, 0), (0, 140, 255), (0, 0, 255)

def draw(frame, box, label, col):
    x1, y1, x2, y2 = box
    cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
    cv2.putText(frame, label, (x1, max(y1 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

class Cam(Node):
    def __init__(self):
        super().__init__("cam_ieee")
        self.br = CvBridge()
        self.model = YOLO("/home/aziz/living_map/yolov8m.pt")
        self.create_subscription(Image, TOPIC, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"listening on {TOPIC}")

    def cb(self, m):
        frame = self.br.imgmsg_to_cv2(m, "bgr8")
        H, W = frame.shape[:2]

        rs = self.model(frame, conf=0.10, classes=[0], imgsz=960, verbose=False)
        for r in rs:
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                conf, name = float(b.conf[0]), self.model.names[int(b.cls[0])]
                if name == "person":
                    lying = (x2 - x1) > (y2 - y1)
                    lab, col = ("person in danger", ORANGE) if lying else ("safe person", GREEN)
                else:
                    lab, col = name, (255, 255, 0)
                draw(frame, (x1, y1, x2, y2), f"{lab} {conf:.2f}", col)
                self.get_logger().info(f"{lab} {conf:.2f}")

        self.get_logger().info(f"yolo boxes: {sum(len(r.boxes) for r in rs)}")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, (0, 0, 245), (180, 25, 255))
        orange = cv2.inRange(hsv, (5, 150, 180), (25, 255, 255))
        mask = cv2.bitwise_or(white, orange)
        mask[: int(H * 0.30), :] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) > 60:
                x, y, w, h = cv2.boundingRect(c)
                draw(frame, (x, y, x + w, y + h), "fire", RED)

        cv2.imshow("cam_ieee", frame)
        cv2.imshow("fire_mask", mask)
        cv2.waitKey(1)

rclpy.init()
node = Cam()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
cv2.destroyAllWindows()
node.destroy_node()
rclpy.shutdown()
