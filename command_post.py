import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPost(Node):
    def __init__(self):
        super().__init__("command_post")
        self.events = []
        self.create_subscription(String, "/command_post/events", self.cb, 10)
        self.pub = self.create_publisher(String, "/command_post/mission", 10)
        self.get_logger().info("command post up")

    def cb(self, m):
        e = json.loads(m.data)
        self.events.append(e)
        live = [x for x in self.events if not x["stale"]]
        mission = {
            "goto": [x for x in live if x["type"] == "victim"],
            "avoid": [x for x in live if x["type"] == "fire"],
        }
        self.pub.publish(String(data=json.dumps(mission)))
        self.get_logger().info(
            f"MISSION goto={len(mission['goto'])} avoid={len(mission['avoid'])}")

rclpy.init()
rclpy.spin(CommandPost())
