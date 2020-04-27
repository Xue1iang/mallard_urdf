# mallard_urdf
Mallards simualtaion in Gazebo with urdf robot description. It uses freefloating_gazebo package 
(a lot of elemnts are actually borrowed from frefloating_gazebo_demo pakcage 
https://github.com/freefloating-gazebo/freefloating_gazebo_demo). Using demo is easier to setup control thrusters.
So far I maneged to to make the physics working such as buoyancy force, drag and control thrusters.


The <compensation> tag inside <buoyancy> plugin is a ratio of buoyancy force to gravity (weight); 
i.e. provides percentage of buoyancy force with respect to gravity. If less then 1 the object will sink if bigger it will float.

To create *.rviz config file, launch urdf without args, change frame to (usually) base_link and add robot description in rviz, then save rviz config file. 
Ammend launch file to add config as arg."

Important:
- add inside bash line for Gazebo path, so it can find the worlds folder:
  'export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9/worlds/:${GAZEBO_RESOURCE_PATH}'
- seting up proper name space inside the launch file so freefloating physics can work. 
  In original package it was <group ns="vectored_auv"> or to control thrusters it is <group ns="thrusters">
