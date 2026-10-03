import os, json, time, math
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.image_pb2 import Image as GzImage

IMG = "/writer/image"
TTL = {"fire": 300, "victim": 600}
MIN_PIX, MIN_DIST = 200, 5.0
FOV_RGB, FOV_D = 1.204, 1.274
DW, DH = 640, 480

class Writer(Node):
    def __init__(self):
        super().__init__("writer")
        self.pos, self.yaw, self.depth, self.beacons = (0.0, 0.0), 0.0, None, []
        self.pub = self.create_publisher(String, "/beacon/drop", 10)
        self.create_subscription(Image, IMG, self.cb, qos_profile_sensor_data)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, "/world/mine_tunnel/dynamic_pose/info", self.pose_cb)
        self.gz.subscribe(GzImage, "/depth_camera", self.depth_cb)
        self.get_logger().info("writer up")

    def pose_cb(self, msg):
        for p in msg.pose:
            if p.name == "x500_depth_0":
                self.pos = (p.position.x, p.position.y)
                o = p.orientation
                self.yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y ** 2 + o.z ** 2))

    def depth_cb(self, msg):
        self.depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()

    def locate(self, mask, W, H):
        if self.depth is None:
            return None
        vs, us = np.nonzero(mask)
        u, v = us.mean(), vs.mean()
        fx_rgb = (W / 2) / math.tan(FOV_RGB / 2)
        tx, ty = (u - W / 2) / fx_rgb, (v - H / 2) / fx_rgb
        fx_d = (DW / 2) / math.tan(FOV_D / 2)
        ud, vd = int(DW / 2 + fx_d * tx), int(DH / 2 + fx_d * ty)
        if not (2 <= ud < DW - 2 and 2 <= vd < DH - 2):
            return None
        w = self.depth[vd - 2:vd + 3, ud - 2:ud + 3]
        w = w[np.isfinite(w) & (w > 0.2) & (w < 19.0)]
        if w.size == 0:
            return None
        d = float(np.median(w))
        bx, by = d, -d * tx
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.pos[0] + c * bx - s * by, self.pos[1] + s * bx + c * by)

    def drop(self, kind, tpos):
        for b in self.beacons:
            if b["type"] == kind and math.dist(b["pos"], tpos) < MIN_DIST:
                return
        b = {"type": kind, "pos": [round(tpos[0], 2), round(tpos[1], 2)],
             "t": int(time.time()), "ttl": TTL[kind]}
        self.beacons.append(b)
        self.pub.publish(String(data=json.dumps(b)))
        open(os.path.expanduser("~/living_map/beacons.jsonl"), "a").write(json.dumps(b) + "\n")
        self.get_logger().info(f"BEACON DROPPED {b}")

    def cb(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3].astype(int)
        r, g, b = img[..., 0], img[..., 1], img[..., 2]
        for kind, mask in (("fire", (r > 120) & (r > 2 * g) & (r > 2 * b)),
                           ("victim", (b > 60) & (b > 2 * r) & (b > 1.3 * g))):
            if mask.sum() > MIN_PIX:
                t = self.locate(mask, m.width, m.height)
                if t:
                    self.drop(kind, t)

rclpy.init()
rclpy.spin(Writer())
