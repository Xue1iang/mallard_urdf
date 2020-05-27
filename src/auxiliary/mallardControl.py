#!/usr/bin/env python

   # safe divide
def safe_div(x, y):
    if y == 0:
        # print 'zero divide warning'
        return 0
    return x/y

def proportional(pos, goal, vel, goal_vel, kp, kd, limit):
    ctrl_input = (goal - pos)*kp + (goal_vel - vel)*kd
    if abs(ctrl_input) > limit:
        ctrl_input = safe_div(ctrl_input, abs(ctrl_input))*limit
        print('position limit hit')
    return ctrl_input


