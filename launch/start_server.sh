#!/bin/bash

CARLA_SCRIPT=$HOME/trunk/carla/CARLA_0.7.1/CarlaUE4.sh

FPS=10.0
SETTINGS_FILE_PATH=$HOME/orb_slam_catkin_ws/src/carla_to_ros/config/CarlaSettings.ini

#NOTE(alexmillane): Setting file here seems to be replaced by the client

eval $CARLA_SCRIPT /Game/Maps/Town02 \
                   -carla-server \
                   -carla-settings="$SETTINGS_FILE_PATH" \
                   -fps=$FPS \
                   -windowed \
                   -ResX=800 \
                   -ResY=600

#                   -benchmark \
