# mallard_urdf
Mallards simualtaion in Gazebo with urdf description. So far I maneged to to make the physics working 
such as buoyancy force and drag. It uses control provided from freefloating. 

Important:
- add inside bash line for Gazebo path, so it can find the worlds folder:
  'export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9/worlds/:${GAZEBO_RESOURCE_PATH}'
- seting up proper name space inside the launch file so freefloating physics can work. 
  In original package it was <group ns="vectored_auv"> or to control thrusters it is <group ns="thrusters">
