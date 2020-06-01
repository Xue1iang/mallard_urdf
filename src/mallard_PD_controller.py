#!/usr/bin/env python

import math
import rospy
import numpy as np
import tf.transformations as tft
import auxiliary.mallardControl as control
from std_msgs.msg import Float64, Float64MultiArray
from mallard_urdf.cfg import MtwoParamConfig
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import time
from statistics import mean,stdev



# variables for velocity calculations:
time_prev = 0
x_prev = 0
y_prev = 0
psi_prev = 0
# varaibles to store goals:
goals_received = False
x_goal       = 0
y_goal       = 0
psi_goal     = 0
x_vel_goal   = 0
y_vel_goal   = 0
psi_vel_goal = 0

# simulation variables:
a_sim=1.0556
b_sim=1.1955
linear_scale=1
angular_scale=1
# inputs to simulation:
thruster_1 = 0
thruster_2 = 0
thruster_3 = 0
thruster_4 = 0
# dictionary to store controller parameters
param = dict(kp=5, kd=1, kp_psi=1.5, kd_psi=0.5,lim=1.4, lim_psi=0.7)

# ----- Functions -----

# required to pass control forces into simulation
def thruster_ctrl_msg():
    global thruster_1,thruster_2,thruster_3,thruster_4
    msg = JointState()
    msg.header = Header()
    msg.name = ['x_thr_left','x_thr_right','y_thr_left','y_thr_right']
    msg.position = []
    msg.velocity = []
    msg.effort = [thruster_1,thruster_2,thruster_4,thruster_3]
    return msg


# ------ Callbacks -----

# Publishing node: mallard_goal_selector.py, topic: /mallard/goals
def goal_callback(array):
    global goals_received, x_goal, y_goal,psi_goal
    global x_vel_goal,y_vel_goal,psi_vel_goal
    
    # goals_received flag has always value (bool) published
    goals_received = array.data[0]

    # Get goal values if data is available
    if (goals_received == True):
        x_goal       = array.data[1]
        y_goal       = array.data[2]
        psi_goal     = array.data[3]
        x_vel_goal   = array.data[4]
        y_vel_goal   = array.data[5]
        psi_vel_goal = array.data[6]

# Publishing node: hector_slam, topic: /slam_out_pose
def slam_callback(msg):
    global time_prev,x_prev,y_prev,psi_prev
    global thruster_1, thruster_2, thruster_3, thruster_4
    
    # ----- Goals recived? -----
    if(goals_received == False):
        thruster_1 = 0
        thruster_2 = 0
        thruster_3 = 0
        thruster_4 = 0
        pub_velocity.publish(thruster_ctrl_msg())
        # print("No goals received, idle. ")
        # Turn off thrustrs and exit callback
        return

    # ----- Position and Velocity -----
    # Get current position, orientation and time:
    x   = msg.pose.position.x
    y   = msg.pose.position.y
    psi = tft.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,\
                                     msg.pose.orientation.z,msg.pose.orientation.w])[2]
    time = (msg.header.stamp.secs + msg.header.stamp.nsecs * 0.000000001)
    # Derive velocities from slam data:
    time_diff = time - time_prev
    x_vel   = control.get_velocity(x,x_prev,time_diff)
    y_vel   = control.get_velocity(y,y_prev,time_diff)
    psi_vel = control.get_velocity(psi,psi_prev,time_diff)

    # ----- Control -----
    #  Get forces in global frame using PD controller
    x_global_ctrl   = control.proportional(x, x_goal, x_vel, x_vel_goal, param['kp'], param['kd'], param['lim'])
    y_global_ctrl   = control.proportional(y, y_goal, y_vel, y_vel_goal, param['kp'], param['kd'], param['lim'])
    psi_global_ctrl = control.proportional_angle(psi, psi_goal,psi_vel,psi_vel_goal, param['kp_psi'], param['kd_psi'], param['lim_psi'])
    # convert into body frame:
    x_body_ctrl =  math.cos(psi)*x_global_ctrl + math.sin(psi)*y_global_ctrl
    y_body_ctrl = -math.sin(psi)*x_global_ctrl + math.cos(psi)*y_global_ctrl
    
    # ----- Simulation -----
    # vector forces scaled in body frame
    x_sim   = (x_body_ctrl)*linear_scale
    y_sim   = (y_body_ctrl)*linear_scale
    psi_sim = (-psi_global_ctrl)*angular_scale

    # thrust allocation
    thruster_1 = 0 + 0.5*x_sim + a_sim*psi_sim
    thruster_2 = 0 + 0.5*x_sim - a_sim*psi_sim
    thruster_3 = 0 - 0.5*y_sim + b_sim*psi_sim
    thruster_4 = 0 - 0.5*y_sim - b_sim*psi_sim
    # Publish forces to simulation (joint_state_publisher message)
    pub_velocity.publish(thruster_ctrl_msg())

    # ----- Next iteratioon -----
    # Before finishing, assign current values to previous ones
    time_prev = time
    x_prev    = x
    y_prev    = y
    psi_prev  = psi

time_p = 0
period_list = []
def timer_callback(event):
    global time_p,period_list

    t = rospy.Time.from_sec(time.time())
    time_now = t.to_sec() + t.to_nsec()

    period = (time_now - time_p)*10**-9
    time_p = time_now

    period_list.append(period)
    if(len(period_list)>2):
        # first element in time stamp; start with second
        period_mean = mean(period_list[1:])
        print("mean period: ",period_mean)
        period_stdev = stdev(period_list[1:])
        output = "{:.9f}".format(period_stdev)
        print("std dev: ",output)
        # print("std devation :" + f"{period_stdev.9f}")

    print("period: " + str(period))
    # print("period list:", period_list)
    
    # print("Timer current_real: ",str(event.current_real))
    print("------------------")
    # print("current_real - last_real: " + str(event.current_real - event.last_real))
    # print("Timer last_duration: " + str(event.last_duration))


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True) 
    # PUBLISHER
    pub_velocity = rospy.Publisher('/mallard/thruster_command',JointState,queue_size=10)

    # SUBSCRIBER
    # rospy.Subscriber("/mallard/goals", Float64MultiArray, goal_callback)  # subscribes to topic "/slam_out_pose"
    rospy.Subscriber("/slam_out_pose",PoseStamped,slam_callback)
    rospy.Subscriber("/mallard/goals",Float64MultiArray,goal_callback)
    # Subscribe to array of goal poses from RVIZ interactive coverage selector
    # dynrecon = Server(MtwoParamConfig)

    rospy.Timer(rospy.Duration(0.1), timer_callback,oneshot=False)

    rospy.spin()