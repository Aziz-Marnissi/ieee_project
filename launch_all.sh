#!/bin/bash
S=ieee
T=/world/mine_tunnel/model/x500_depth_0/link/camera_link/sensor/IMX214/image
R="source /opt/ros/humble/setup.bash"
tmux kill-session -t $S 2>/dev/null; pkill -f "gz sim"; pkill -f bin/px4; pkill -f "python3 .*cam_ieee.py"; sleep 2
tmux new-session -d -s $S -n sim "cd ~/PX4-Autopilot && PX4_GZ_WORLD=mine_tunnel make px4_sitl gz_x500_depth"
tmux new-window -t $S -n bridge "sleep 40; $R; ros2 run ros_gz_image image_bridge $T --ros-args -r $T:=/writer/image"
tmux new-window -t $S -n execbridge "sleep 40; $R; ros2 run ros_gz_bridge parameter_bridge /exec/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry /exec/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"
tmux new-window -t $S -n cam "sleep 50; $R; python3 ~/living_map/cam_ieee.py 2>&1 | grep -v -i matplotlib"
tmux new-window -t $S -n qgc "sleep 30; ~/Downloads/QGroundControl.AppImage"
tmux attach -t $S
