import json, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt

LAT0, LON0 = 36.8065, 10.1815
M_LAT = 111320.0
M_LON = 111320.0 * math.cos(math.radians(LAT0))
STYLE = {"victim": ("o", "tab:blue"), "fire": ("^", "tab:red")}

class LiveMap(Node):
    def __init__(self):
        super().__init__("live_map")
        self.events = {}
        self.create_subscription(String, "/command_post/events", self.cb, 10)

    def cb(self, m):
        e = json.loads(m.data)
        key = (e["type"], round(e["lat"], 5), round(e["lon"], 5))
        self.events[key] = e

rclpy.init()
node = LiveMap()
plt.ion()
fig, ax = plt.subplots(figsize=(8, 5))
while plt.fignum_exists(fig.number):
    rclpy.spin_once(node, timeout_sec=0.05)
    ax.clear()
    now = time.time()
    seen = set()
    for (kind, lat, lon), e in node.events.items():
        x, y = (lon - LON0) * M_LON, (lat - LAT0) * M_LAT
        age = now - e["t"]
        stale = age > e["ttl"]
        conf = 0.15 if stale else max(0.3, math.exp(-age / e["ttl"]))
        mk, col = STYLE.get(kind, ("s", "gray"))
        ax.scatter(x, y, marker=mk, c=col, s=200, alpha=conf,
                   label=kind if kind not in seen else None)
        seen.add(kind)
        ax.annotate(f"{kind}\n{int(age)}s{' STALE' if stale else ''}", (x, y),
                    textcoords="offset points", xytext=(8, 8), fontsize=8)
    ax.set_title(f"Command Post - Live Map ({len(node.events)} events)\n"
                 f"origin {LAT0}, {LON0}")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_aspect("equal"); ax.grid(True)
    if seen: ax.legend(loc="upper right")
    plt.pause(0.5)
