#!/usr/bin/env bash

cd rosbags

name=$1
rosbag record -O $name /mallard/goals /slam_out_pose



# rosbag record -O test_python_execute.bag /mallard/goals /slam_out_pose
