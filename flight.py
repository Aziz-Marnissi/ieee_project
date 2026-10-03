import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition

Z = -1.5
# (north, east, yaw) in PX4 NED. north = world y, east = world x - 1, yaw pi/2 = facing world +x
WPS = [(0.0, 0.0, math.pi/2), (0.0, 11.0, math.pi/2), (6.0, 11.0, 0.0),
       (0.0, 11.0, math.pi/2), (0.0, 18.0, math.pi/2)]
PAUSE, REACH = 3.0, 0.5

class Flight(Node):
    def __init__(self):
        super().__init__("flight")
        q = qos_profile_sensor_data
        self.om = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", q)
        self.sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", q)
        self.cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", q)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_pos, q)
        self.pos, self.i, self.n, self.t_reach = None, 0, 0, None
        self.sp_cur = None
        self.create_timer(0.1, self.loop)

    def now(self): return int(self.get_clock().now().nanoseconds / 1000)

    def on_pos(self, m): self.pos = (m.x, m.y, m.z)

    def send_cmd(self, c, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp, m.command, m.param1, m.param2 = self.now(), c, float(p1), float(p2)
        m.target_system = m.source_system = 1
        m.target_component = m.source_component = 1
        m.from_external = True
        self.cmd.publish(m)

    def loop(self):
        o = OffboardControlMode()
        o.timestamp, o.position = self.now(), True
        self.om.publish(o)
        if self.i >= len(WPS):
            n, e, yaw = WPS[-1]
        else:
            n, e, yaw = WPS[self.i]
        s = TrajectorySetpoint()
        s.timestamp = self.now()
        if self.sp_cur is None and self.pos: self.sp_cur = [self.pos[0], self.pos[1]]
        if self.sp_cur:
            dx, dy = n - self.sp_cur[0], e - self.sp_cur[1]
            d = math.hypot(dx, dy); step = min(d, 0.04)
            if d > 1e-6: self.sp_cur[0] += dx/d*step; self.sp_cur[1] += dy/d*step
            n, e = self.sp_cur
        s.position = [float(n), float(e), Z]
        s.yaw = float(yaw)
        self.sp.publish(s)
        self.n += 1
        if self.n == 100:
            self.send_cmd(176, 1, 6)   # offboard mode
            self.send_cmd(400, 1)      # arm
        if self.pos and self.i < len(WPS) and self.n > 50:
            if math.hypot(self.pos[0] - WPS[self.i][0], self.pos[1] - WPS[self.i][1]) < REACH and abs(self.pos[2] - Z) < 0.5:
                if self.t_reach is None:
                    self.t_reach = time.time()
                    self.get_logger().info(f"WP {self.i} reached {WPS[self.i]}")
                elif time.time() - self.t_reach > PAUSE:
                    self.i += 1
                    self.t_reach = None
                    if self.i >= len(WPS): self.get_logger().info("PATH DONE, holding")

rclpy.init()
rclpy.spin(Flight())
