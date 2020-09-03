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


# slam position and velocity variables:
x = 0
y = 0
psi = 0
x_vel = 0
y_vel = 0
psi_vel = 0
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
x_acc_goal   = 0
y_acc_goal   = 0
goal_met     = False

# Step variables:
goal_number = 0
step_counter = 0
loop_period = 0.1
step_seconds = 3 # step interval in seconds
step_ctrl_input = 0
step_range = 2.78


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
param       = dict(kp=5, kd=1, kp_psi=1.5, kd_psi=0.5,lim=1.4, lim_psi=0.7)

# ----- Functions -----
def thruster_ctrl_msg():
    # required to pass control forces into simulation
    global thruster_1,thruster_2,thruster_3,thruster_4
    msg = JointState()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['x_thr_left','x_thr_right','y_thr_left','y_thr_right']
    msg.position = []
    msg.velocity = []
    msg.effort = [thruster_1,thruster_2,thruster_4,thruster_3]
    return msg


# ------ Callbacks -----
# GOALS
def goal_callback(array):
    # Publishing node: mallard_goal_selector.py, topic: /mallard/goals
    global goals_received, x_goal, y_goal,psi_goal
    global x_vel_goal,y_vel_goal,psi_vel_goal
    global x_acc_goal, y_acc_goal
    global goal_number
    
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
        x_acc_goal   = array.data[7]
        y_acc_goal   = array.data[8]
        goal_number  = array.data[9]
        # goal_number = int(goal_number)


# SLAM pose
def slam_callback(msg):
    # Publishing node: hector_slam, topic: /slam_out_pose
    global x,y,psi,x_vel,y_vel,psi_vel # slam position and derived velocity
    global time_prev,x_prev,y_prev,psi_prev # variables for next iteration

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

    # control place holder if timer_callback inactive
    
    # ----- for next iteratioon -----
    time_prev = time
    x_prev    = x
    y_prev    = y
    psi_prev  = psi

# CONTROLLER
def control_callback(event):
    # rospy.Timer() callback trigered by Duration(event).
    # Actual control is done here. The 'event' is the rospy.Timer() duration period, can be used 
    # for trubleshooting. To test how often is executed use: $ rostopic hz /mallard/thruster_commands.

    global thruster_1, thruster_2, thruster_3, thruster_4 # control forces
    global step_number,step_counter,step_ctrl_input

    #  Get forces in global frame using PD controller
    if(goals_received == True):
        # --- Step control ---
        if(goal_number == 1):
            step_number = step_seconds/loop_period
            if(step_counter >= step_number):
                if (step_ctrl_input == 0):
                    # toggle input
                    step_ctrl_input = step_range
                else:
                    step_ctrl_input = 0
                step_counter = 0
                # apply randomised period each time it swaps
                # between 2 to 10 seconds
                # step_seconds = randint(1,10)
                # step_number = step_seconds/loop_period
            else:
                step_counter += 1
            # print("Goal number: ",goal_number)
            print("Now the step_seconds: ",step_seconds,"|| step input: ", step_ctrl_input)
        else:
            print("Goal reached; goal number: ", goal_number)

        # ----- control -----        
        x_global_ctrl   = control.proportional(x, x_goal, x_vel, x_vel_goal, param['kp'], param['kd'], param['lim'])
        y_global_ctrl   = control.proportional(y, y_goal, y_vel, y_vel_goal, param['kp'], param['kd'], param['lim'])
        psi_global_ctrl = control.proportional_angle(psi, psi_goal,psi_vel,psi_vel_goal, param['kp_psi'], param['kd_psi'], param['lim_psi'])

        # convert into body frame:
        x_body_ctrl =  math.cos(psi)*x_global_ctrl + math.sin(psi)*y_global_ctrl
        y_body_ctrl = -math.sin(psi)*x_global_ctrl + math.cos(psi)*y_global_ctrl

        # override x-body control with step function:
        if(goal_number==1):
            print("executing step")
            x_body_ctrl = step_ctrl_input

        elif(goal_number>=2):
            x_body_ctrl = 0
            y_body_ctrl = 0
            psi_global_ctrl = 0
            #  maitain position at goal one
            print("Finished! Mallard is idling.")
        # test
        print("Step Input: ", step_ctrl_input)
        
        # ----- simulation -----
        # vector forces scaled in body frame
        x_sim   = (x_body_ctrl)*linear_scale
        y_sim   = (y_body_ctrl)*linear_scale
        psi_sim = (-psi_global_ctrl)*angular_scale

        # ----- thrust allocation -----
        thruster_1 = 0 + 0.5*x_sim + a_sim*psi_sim
        thruster_2 = 0 + 0.5*x_sim - a_sim*psi_sim
        thruster_3 = 0 - 0.5*y_sim + b_sim*psi_sim
        thruster_4 = 0 - 0.5*y_sim - b_sim*psi_sim

        # Publish forces to simulation (joint_state_publisher message)
        pub_velocity.publish(thruster_ctrl_msg())
    else:
        # ----- idle if no goals -----
        thruster_1 = 0
        thruster_2 = 0
        thruster_3 = 0
        thruster_4 = 0
        pub_velocity.publish(thruster_ctrl_msg())

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True) 
    # PUBLISHER
    pub_velocity = rospy.Publisher('/mallard/thruster_command',JointState,queue_size=10)

    # SUBSCRIBER
    rospy.Subscriber("/slam_out_pose",PoseStamped,slam_callback)
    rospy.Subscriber("/mallard/goals",Float64MultiArray,goal_callback)
    rospy.Timer(rospy.Duration(0.1), control_callback,oneshot=False)

    rospy.spin()