#!/usr/bin/env python
import kguseful
import tf.transformations as tft
import math


# velocity from displacement and time
def vel_fun(dv, dt):
    dvdt = [kguseful.safe_div(i, j) for i, j in zip(dv, dt)]
    vel = kguseful.safe_div(sum(dvdt), float(len(dvdt)))
    return vel


def velramp(t, velabs, xy0, xyg, tr,name):
    d = xyg-xy0
    vel = velabs*kguseful.safe_div(d, abs(d))  # avoid zero division stability
    a = vel/tr
    tv = kguseful.safe_div((d-tr*vel), vel)
    if tv > 0:
        if t <= tr:
            if (name == "x") : print(name + "-ramp acc is +a")
            ades = a
            veldes = t*a
            xydes = 0.5*veldes*t + xy0
        elif t > tr and t < tr+tv:
            if (name == "x") : print(name + "-ramp acc is 0")
            ades = 0
            veldes = vel
            xydes = 0.5*vel*tr + (t-tr)*vel + xy0
        elif t > tr + tv and t < 2*tr + tv:
            if (name == "x") : print(name + "-ramp acc is -a")
            ades = -a
            veldes = vel-(t-tr-tv)*a
            xydes = 0.5*vel*tr + tv*vel + veldes*(t-tr-tv) + 0.5*(vel-veldes)*(t-tr-tv) + xy0
        else:
            if (name == "x") : print(name + "-ramp acc is out of bound: 0")
            ades = 0 
            veldes = 0
            xydes = xyg
        
    elif tv <= 0:
        
        if (a == 0):
            tg = math.sqrt((4*d)/0.001)
            if (name == "x") : print (name + "-triangle. Error, cant divide by 0, t: ",t," tg: ",tg," and d:",d)
        else:
            if (name == "x") : print(name + "-triangle. Standard computation")
            tg = math.sqrt((4*d)/a)
        if t <= 0.5*tg:
            if (name == "x") : print(name + "-triangle acc is +a")
            ades = a
            veldes = a*t
            xydes = 0.5*veldes*t + xy0
            # print(name, ": +a in triangle")
        elif t > 0.5*tg and t < tg:
            if (name == "x") : print(name + "-triangle acc is -a")
            ades = -a
            veldes = 0.5*tg*a - (t - 0.5*tg)*a
            vm2 = 0.5*tg*a
            xydes = vm2*0.5*tg - 0.5*(tg-t)*vm2 + xy0
        else:
            if (name == "x") : print(name + "-tirnagle acc is out of bound: 0")
            ades = 0
            veldes = 0
            xydes = xyg
            # print(name, ": outside of triangle")

    return xydes, veldes,ades


# desired x and y values
def desxy_fun(goal, zero, t_goalf, t_nowf):
    if t_nowf < t_goalf:
        des = zero + (goal-zero)*(kguseful.safe_div(t_nowf, t_goalf))
    else:
        des = goal
    return des


# desired linear velocities
def desvel_fun(goal, zero, t_goalf, t_nowf):
    if t_nowf < t_goalf:
        des = kguseful.safe_div((goal - zero), t_goalf)
    else:
        des = 0
    return des


# desired quaternion
def despsi_fun(goal, t_gpsi, q0f, t_nowf):
    if t_nowf < t_gpsi:
        ratio = kguseful.safe_div(t_nowf, t_gpsi)
        des = tft.quaternion_slerp(q0f, goal, ratio)
    else:
        des = goal
    return des


# desired angular velocity
def desvelpsi_fun(edf, t_goalf, t_nowf, vel_request):
    if t_nowf < t_goalf:
        des = (edf / abs(edf))*vel_request
    else:
        des = 0
    return des


# xy controller with limit - displacement and velocity
def cont_fun(xy, des, vel, veldes, kp, kd, lim):
    fp = (des - xy) * kp
    fd = (veldes - vel) * kd
    f_nav = fp + fd
    if abs(f_nav) > lim:
        f_nav = kguseful.safe_div(f_nav, abs(f_nav)) * lim
        print('xy limit hit')
    return f_nav


# psi controller with limit - displacement only
def contpsi_fun(q, des, vel, veldes, kp, kd, lim):
    err_psi = kguseful.err_psi_fun(q, des)
    fp = err_psi * kp
    fd = (veldes - vel) * kd
    f_nav = fp + fd
    if abs(f_nav) > lim:
        f_nav = kguseful.safe_div(f_nav, abs(f_nav)) * lim
        print('psi limit hit')
    return f_nav
