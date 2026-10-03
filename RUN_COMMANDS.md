# Commands

Install the world and models into PX4 first:
cp worlds/mine_tunnel.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp -r models/* ~/PX4-Autopilot/Tools/simulation/gz/models/

# T0: sim, bridges, QGC, camera
~/living_map/launch_all.sh        # wait ~60 s

# T1: DDS agent (check: ros2 topic list | grep -c fmu/in  ->  ~38)
MicroXRCEAgent udp4 -p 8888

# T2: gateway
source /opt/ros/humble/setup.bash && python3 ~/living_map/gateway.py

# T3: command post
source /opt/ros/humble/setup.bash && python3 ~/living_map/command_post.py

# T4: beacon RF
source /opt/ros/humble/setup.bash && python3 ~/living_map/beacon_rf.py

# T5: rover
source /opt/ros/humble/setup.bash && python3 ~/living_map/rover_exec.py

# T6: writer (wait for "writer up")
source /opt/ros/humble/setup.bash && export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python && python3 ~/living_map/writer.py

# T7: flight (after "writer up")
source /opt/ros/humble/setup.bash && source ~/ws_sensor_combined/install/setup.bash && python3 ~/living_map/flight.py
