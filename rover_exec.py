import json, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LAT0, LON0 = 36.8065, 10.1815
M_LAT = 111320.0
M_LON = 111320.0 * math.cos(math.radians(LAT0))
SPAWN = (-3.0, 0.0)   # rover spawn in world frame
STOP_DIST = 1.5

class RoverExec(Node):
    def __init__(self):
        super().__init__("rover_exec")
        self.targets = []      # from mission briefing (odom frame)
        self.beacons = {}      # id -> (x, y, kind), odom frame, read over RF
        self.pose = None
        self.mode = None
        self.create_subscription(String, "/gateway/briefing", self.on_mission, 10)
        self.create_subscription(String, "/beacon/rf", self.on_rf, 10)
        self.create_subscription(Odometry, "/exec/odom", self.on_odom, 10)
        self.pub = self.create_publisher(Twist, "/exec/cmd_vel", 10)
        self.create_timer(0.1, self.loop)
        self.get_logger().info("rover exec up")

    def on_mission(self, m):
        goto = json.loads(m.data)["goto"]
        self.targets = [((g["lon"] - LON0) * M_LON - SPAWN[0],
                         (g["lat"] - LAT0) * M_LAT - SPAWN[1]) for g in goto]
        self.get_logger().info(f"MISSION received: {len(self.targets)} target(s) {self.targets}")

    def on_rf(self, m):
        if self.pose is None:
            return
        b = json.loads(m.data)
        bx, by = b["p"][0] - SPAWN[0], b["p"][1] - SPAWN[1]
        if math.hypot(bx - self.pose[0], by - self.pose[1]) > b["r"]:
            return  # out of RF range
        if b["id"] not in self.beacons:
            self.get_logger().info(f"RF beacon read: {b['id']} {b['k']} age={b['age']}s")
        self.beacons[b["id"]] = (bx, by, b["k"])

    def on_odom(self, o):
        p, q = o.pose.pose.position, o.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def loop(self):
        if self.pose is None or not self.targets:
            return
        x, y, yaw = self.pose
        cands = [(bx, by) for bx, by, k in self.beacons.values() if k == "victim"]
        mode = "BEACON" if cands else "MISSION"
        if mode != self.mode:
            self.mode = mode
            self.get_logger().info(f"navigating by {mode}")
        pts = cands or self.targets
        tx, ty = min(pts, key=lambda t: math.hypot(t[0] - x, t[1] - y))
        d = math.hypot(tx - x, ty - y)
        cmd = Twist()
        if d > STOP_DIST:
            err = math.atan2(ty - y, tx - x) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            cmd.angular.z = max(-1.0, min(1.0, 1.5 * err))
            cmd.linear.x = 0.5 if abs(err) < 0.5 else 0.0
        else:
            self.get_logger().info(f"ARRIVED at victim (guided by {mode})", throttle_duration_sec=5.0)
        self.pub.publish(cmd)

rclpy.init()
rclpy.spin(RoverExec())
