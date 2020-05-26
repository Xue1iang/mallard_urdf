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

# Variables for slam:
time_prev = 0
x_prev = 0
y_prev = 0
psi_prev = 0
q_prev = [0,0,0,0]
# varaibles for goal:
thrusters_on = False
x_goal       = 0
y_goal       = 0
psi_goal     = 0
x_vel_goal   = 0
y_vel_goal   = 0
psi_vel_goal = 0

# Functions
def get_velocity (current_pos,previous_pos,time):
    return (current_pos - previous_pos)/time

def get_ang_velocity(current_ang,previous_ang,time):
    return (current_ang - previous_ang)/time

# def get_ang_velocity_quot (current_quot, previous_quot, time):
#     '''
#     method that takes quoternions, time interval between them
#     and returns angular velocity (euler angle per second)
#     '''
#     quot_diff = tft.quaternion_multiply(current_quot,[-previous_quot[0],-previous_quot[1],-previous_quot[2],previous_quot[3]])
#     angle = tft.euler_from_quaternion(quot_diff)
#     return angle[2]/time

def goal_callback(array):
    global thrusters_on, x_goal, y_goal,psi_goal
    global x_vel_goal,y_vel_goal,psi_vel_goal

    thrusters_on = array.data[0]

    if(len(array.data) > 1):
        x_goal       = array.data[1]
        y_goal       = array.data[2]
        psi_goal     = array.data[3]

        x_vel_goal   = array.data[4]
        y_vel_goal   = array.data[5]
        psi_vel_goal = array.data[6]

        print("thrusters_on flag: " + str(thrusters_on) + "x_goal:" + str(x_goal) + "y_goal:" + str(y_goal) + "psi_goal:" + str(psi_goal))
        print("x_vel_goal:" + str(x_vel_goal) + "y_vel_goal:" + str(y_vel_goal) + "psi_vel_goal:" + str(psi_vel_goal))
    else:    
        print("No goals received")

def slam_callback(msg):
    global time_prev,x_prev,y_prev,psi_prev,q_prev

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
    # psi_vel = get_ang_velocity(q,q_prev,time_diff)

    # print(get_ang_velocity.__doc__) 
    # print("angle :"+ str(psi)+" angular velocity: " + str(psi_vel))

    # HISTORY - assign current values as previous ones
    time_prev = time
    x_prev    = x
    y_prev    = y
    psi_prev  = psi
    # q_prev    = q

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