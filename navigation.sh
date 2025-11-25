#!/bin/bash

set -e

# Ctrl+C 時，自動 kill 所有子 process
trap "echo 'Stopping...'; pkill -P $$; exit 0" SIGINT

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02

ros2 launch turtlebot3_navigation2 navigation2.launch.py  \
    map:=/media/sf_vmbox/turtle/map.yaml \
    params_file:=/media/sf_vmbox/turtle/burger.yaml \
    use_rviz:=False

