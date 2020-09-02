#!/usr/bin/env bash

cd rosbags

name=$1
rosbag record -O $name /mallard/goals /gazebo/model_states /mallard/thruster_command



