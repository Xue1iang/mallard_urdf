# mallard_urdf
Mallards simualtaion in Gazebo with urdf robot description. It uses freefloating_gazebo package 
(a lot of elemnts are actually borrowed from frefloating_gazebo_demo package 
https://github.com/freefloating-gazebo/freefloating_gazebo_demo). Using demo is easier to setup control thrusters.

To launch simulation:
1. $ roslaunch mallard_urdf mallard_gazebo.launch 
   - this launches gazebo world and Mallard urdf with freefloating_gazebo plugin. also launches Lidar plugin (need to adjust noise mean and stdev)
2. $ roslaunch mallard_urdf mallard_teleop_and_control.launch 
   - launches SLAM, control (Keir's for testing)and path planner.



The <compensation> tag inside <buoyancy> plugin is a ratio of buoyancy force to gravity (weight); 
i.e. provides percentage of buoyancy force with respect to gravity. If less then 1 the object will sink if bigger it will float.


Important:
- add inside bash line for Gazebo path, so it can find the worlds folder:
  'export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9/worlds/:${GAZEBO_RESOURCE_PATH}'
- seting up proper name space inside the launch file so freefloating physics can work. 
  In original package it was 'group ns="vectored_auv"' or to control thrusters it is 'group ns="thrusters"'


  To do:
  - add lidar and see how it balances. I added bit of buoyancy force due to the uneven weight distribution. so <compensation> is set to 1.2 
    (instead of 1.1). i.e buoyancy/weight = 1.2.
  - see if you can add a function that represents thruster characteristics found experimentally.
  
  
Note on creating rviz config.
To create *.rviz config file, launch urdf without args, change frame to (usually) base_link and add robot description in rviz, then save rviz config file. Ammend launch file to add config as arg."

Edit, 13th May 2020:
  - launch gazebo first and 'play' the simulation. Then launch control and teleop, so SLAM can read Lidar values after simulation starts playing.
  -
