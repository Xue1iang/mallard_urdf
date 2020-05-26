#!/usr/bin/env python

import math
import rospy
import numpy as np
import collections as coll
import tf.transformations as tft
import auxiliary.kglocal as kglocal
import auxiliary.kguseful as kguseful
from std_msgs.msg import Float64, Float64MultiArray
from mallard_urdf.cfg import MtwoParamConfig
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Variables for slam:
time_prev = 0
x_prev = 0
y_prev = 0
psi_prev = 0
# varaibles for goal:
thrusters_on = False
x_goal       = 0
y_goal       = 0
psi_goal     = 0
x_vel_goal   = 0
y_vel_goal   = 0
psi_vel_goal = 0

#  simulation variables:
a_sim=1.0556
b_sim=1.1955
linear_scale=2
angular_scale=2

thruster_1 = 0
thruster_2 = 0
thruster_3 = 0
thruster_4 = 0

param = dict(kp=5, kd=1, kp_psi=1.5, kd_psi=0.5,lim=1.4, lim_psi=0.7)

# Functions
def get_velocity (current_pos,previous_pos,time):
    return (current_pos - previous_pos)/time

def get_ang_velocity(current_ang,previous_ang,time):
    return (current_ang - previous_ang)/time

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

def goal_callback(array):
    global thrusters_on, x_goal, y_goal,psi_goal
    global x_vel_goal,y_vel_goal,psi_vel_goal

    thrusters_on = array.data[0]

    if (thrusters_on == True):
        x_goal       = array.data[1]
        y_goal       = array.data[2]
        psi_goal     = array.data[3]

        x_vel_goal   = array.data[4]
        y_vel_goal   = array.data[5]
        psi_vel_goal = array.data[6]

        # print("thrusters_on flag: " + str(thrusters_on) + "x_goal:" + str(x_goal) + "y_goal:" + str(y_goal) + "psi_goal:" + str(psi_goal))
        # print("x_vel_goal:" + str(x_vel_goal) + "y_vel_goal:" + str(y_vel_goal) + "psi_vel_goal:" + str(psi_vel_goal))

def slam_callback(msg):
    global time_prev,x_prev,y_prev,psi_prev
    global thruster_1, thruster_2, thruster_3, thruster_4
    
    if(thrusters_on == False):
        # Do not compute any further; turn off thrusters
        thruster_1 = 0
        thruster_2 = 0
        thruster_3 = 0
        thruster_4 = 0
        pub_velocity.publish(thruster_ctrl_msg())
        print("No goals received, return. ")
        # print("thruster_1: " + str(thruster_1))
        return

    # Get current position, orientation and time:
    x   = msg.pose.position.x
    y   = msg.pose.position.y
    psi = tft.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,\
                                     msg.pose.orientation.z,msg.pose.orientation.w])[2]
    time = (msg.header.stamp.secs + msg.header.stamp.nsecs * 0.000000001)

    # Derive velocities from slam data:
    time_diff = time - time_prev
    x_vel   = get_velocity(x,x_prev,time_diff)
    y_vel   = get_velocity(y,y_prev,time_diff)
    psi_vel = get_ang_velocity(psi,psi_prev,time_diff)
    
    #  Get forces in global frame using PD controller
    x_global_ctrl   = kglocal.cont_fun(x, x_goal, x_vel, x_vel_goal, param['kp'], param['kd'], param['lim'])
    y_global_ctrl   = kglocal.cont_fun(y, y_goal, y_vel, y_vel_goal, param['kp'], param['kd'], param['lim'])
    psi_global_ctrl = kglocal.cont_fun(psi, psi_goal,psi_vel,psi_vel_goal, param['kp_psi'], param['kd_psi'], param['lim_psi'])
    # print("x_nav_goal: " + str(x_nav_ctrl) + ", y_nav_goal: " + str(y_nav_ctrl) + ", psi_nav_ctrl: " + str(psi_nav_ctrl))
    
    # convert into body frame:
    x_body_ctrl =  math.cos(psi)*x_global_ctrl + math.sin(psi)*y_global_ctrl
    y_body_ctrl = -math.sin(psi)*x_global_ctrl + math.cos(psi)*y_global_ctrl
    
    # ------- Simulation ------------------
    x_sim   = (x_body_ctrl)*linear_scale
    y_sim   = (y_body_ctrl)*linear_scale
    psi_sim = (-psi_global_ctrl)*angular_scale

    thruster_1 = 0 + 0.5*x_sim + a_sim*psi_sim
    thruster_2 = 0 + 0.5*x_sim - a_sim*psi_sim
    thruster_3 = 0 - 0.5*y_sim + b_sim*psi_sim
    thruster_4 = 0 - 0.5*y_sim - b_sim*psi_sim

    # Publish forces to simulation (joint_state_publisher message)
    pub_velocity.publish(thruster_ctrl_msg())

    # Before finishing, assign current values to previous ones
    time_prev = time
    x_prev    = x
    y_prev    = y
    psi_prev  = psi

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

    rospy.spin()