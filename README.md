# The Living Map: TSYP14 (simulation)

Two-robot system for GPS-denied tunnels: a UAV (Writer) detects events and drops beacons, an Outside Network Area (gateway) translates them to GPS for a command post, and a rover (Executor) is briefed and navigates using the beacons.

## Chain
Writer (UAV, `flight.py` + `writer.py`) -> `/beacon/drop` -> `beacon_rf.py` (`/beacon/rf`, TTL aging)
-> `gateway.py` (local -> GPS) -> `command_post.py` (mission) -> `gateway.py` (briefing) -> `rover_exec.py`

## Requirements
Ubuntu, ROS 2 Humble, Gazebo Harmonic, PX4-Autopilot (SITL), Micro XRCE-DDS Agent, px4_msgs workspace.

## Run
1. `~/living_map/launch_all.sh` (sim, bridges, QGC, camera). Wait ~60 s.
2. `MicroXRCEAgent udp4 -p 8888`
3. One terminal each: `gateway.py`, `command_post.py`, `beacon_rf.py`, `rover_exec.py`, `writer.py` (wait for "writer up"), then `flight.py`.

## Simulation stand-ins
Gazebo pose replaces odometry; ROS topic `/beacon/rf` replaces the RF radio.
