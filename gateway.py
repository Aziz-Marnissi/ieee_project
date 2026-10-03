import json, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LAT0, LON0 = 36.8065, 10.1815
M_LAT = 111320.0
M_LON = 111320.0 * math.cos(math.radians(LAT0))

class Gateway(Node):
    def __init__(self):
        super().__init__("outside_gateway")
        self.create_subscription(String, "/beacon/drop", self.cb, 10)
        self.pub = self.create_publisher(String, "/command_post/events", 10)
        self.create_subscription(String, "/command_post/mission", self.mission_cb, 10)
        self.brief = self.create_publisher(String, "/gateway/briefing", 10)
        self.get_logger().info("gateway up")

    def mission_cb(self, m):
        self.brief.publish(m)
        self.get_logger().info(f"BRIEFING relayed to executor: {m.data}")

    def cb(self, m):
        try:
            b = json.loads(m.data)
            x, y = b["pos"]
            b["type"], b["t"], b["ttl"]
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f"bad beacon dropped: {m.data}")
            return
        age = time.time() - b["t"]
        out = {
            "type": b["type"],
            "lat": round(LAT0 + y / M_LAT, 7),
            "lon": round(LON0 + x / M_LON, 7),
            "t": b["t"],
            "ttl": b["ttl"],
            "stale": age > b["ttl"],
            "confidence": round(math.exp(-max(age, 0.0) / b["ttl"]), 2),
        }
        self.pub.publish(String(data=json.dumps(out)))
        self.get_logger().info(f"FORWARDED {out}")

rclpy.init()
rclpy.spin(Gateway())
