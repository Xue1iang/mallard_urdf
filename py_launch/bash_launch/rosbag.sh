#!/usr/bin/env bash

cd rosbags
rosbag record -O test_python_execute.bag /mallard/goals /slam_out_pose

