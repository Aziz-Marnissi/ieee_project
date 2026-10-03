import json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RF_RANGE = 8.0  # m, receivers must be within this distance (enforced by the listener)

class BeaconRF(Node):
    def __init__(self):
        super().__init__("beacon_rf")
        self.beacons = {}
        self.create_subscription(String, "/beacon/drop", self.on_drop, 10)
        self.pub = self.create_publisher(String, "/beacon/rf", 10)
        self.create_timer(1.0, self.broadcast)
        self.get_logger().info(f"beacon RF up (range {RF_RANGE} m)")

    def on_drop(self, m):
        b = json.loads(m.data)
        bid = f"{b['type'][0]}{len(self.beacons)}"
        self.beacons[bid] = b
        self.get_logger().info(f"BEACON ARMED {bid} at {b['pos']}")

    def broadcast(self):
        now = time.time()
        for bid, b in list(self.beacons.items()):
            age = now - b["t"]
            if age > b["ttl"]:
                del self.beacons[bid]
                self.get_logger().info(f"BEACON EXPIRED {bid}")
                continue
            msg = {"id": bid, "k": b["type"], "p": b["pos"], "t": b["t"],
                   "age": int(age), "r": RF_RANGE}
            self.pub.publish(String(data=json.dumps(msg)))

rclpy.init()
rclpy.spin(BeaconRF())
