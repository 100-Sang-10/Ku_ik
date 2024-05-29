#! /bin/bash

# CARLA 시뮬레이터를 새로운 터미널에서 실행
gnome-terminal -- bash -c "cd /opt/carla-simulator/ && ./CarlaUE4.sh -prefernvidia -quality-level=low -fps=10 -windowed -Resx=640 -Resy=480" &

# CARLA 시뮬레이터가 시작될 때까지 대기
sleep 3

# CARLA ROS 브리지를 새로운 터미널에서 실행
gnome-terminal -- bash -c "cd /home/eonsoo/carla-ros-bridge && roslaunch carla_ros_bridge carla_ros_bridge_dream_semester_with_all.launch town:=Town05"&

sleep 3

gnome-terminal -- bash -c "roslaunch preprocessing Lidar.launch"