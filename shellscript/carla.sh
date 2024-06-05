#!/bin/bash

# # CARLA 시뮬레이터를 새로운 터미널에서 실행하고 터미널을 닫음
# gnome-terminal -- bash -c "cd /opt/carla-simulator/ && ./CarlaUE4.sh -prefernvidia -quality-level=low -fps=10 -windowed -Resx=640 -Resy=480; exit" &

# # CARLA 시뮬레이터가 시작될 때까지 대기
# sleep 1

# # CARLA ROS 브리지를 새로운 터미널에서 실행하고 터미널을 닫음
# gnome-terminal -- bash -c "cd /home/eonsoo/carla-ros-bridge && roslaunch carla_ros_bridge carla_ros_bridge_dream_semester_with_all.launch town:=Town05; exit" &

# sleep 1

# 나머지 노드들을 새로운 터미널에서 실행하고 터미널을 유지
gnome-terminal -- bash -c "roslaunch preprocessing Lidar.launch; exec bash" &

sleep 1

gnome-terminal -- bash -c "rosrun behavior_planning behavior_planning_node; exec bash" &

sleep 1

gnome-terminal -- bash -c "roslaunch global_planning global_planning.launch; exec bash" &

sleep 1

gnome-terminal -- bash -c "rosrun point_control point_control; exec bash" &

sleep 1

gnome-terminal -- bash -c "rosrun pedestrian_control pedestrian_control; exec bash" &

sleep 1

gnome-terminal -- bash -c "rosrun dynamic_vehicle_control dynamic_vehicle_1_control; exec bash" &

sleep 1

gnome-terminal -- bash -c "rosrun dynamic_vehicle_control dynamic_vehicle_2_control; exec bash" &

sleep 1

gnome-terminal -- bash -c "rosrun dynamic_vehicle_control dynamic_vehicle_3_control; exec bash"

sleep 1