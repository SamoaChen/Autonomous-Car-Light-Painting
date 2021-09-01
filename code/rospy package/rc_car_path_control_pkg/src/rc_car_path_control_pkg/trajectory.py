#!/usr/bin/env python
from math import sin, cos, atan2, pow, sqrt, pi 
from rc_car_path_control_pkg import l1, trajectory_selection, droplet_radius,  default_start_position, default_T
import numpy as np

#-----FUNCTIONSi
#TRAJECTORY SELECTION FUNCTION
def generate_path(start_position=default_start_position, t=0, T=default_T, activation_no = trajectory_selection):
    end_position_acc = np.array([[1], [1]])
    end_position_dec = np.array([[1], [-1]])

    if activation_no == 1:
        x_di, u_di = line_path_acc(start_position, end_position_acc, t, T)
        return x_di, u_di
    elif activation_no == 2:
        x_di, u_di = line_path_dec(start_position, end_position_dec, t, T)
        return x_di, u_di
    elif activation_no == 3:
        x_di, u_di = droplet_path(start_position, t, T)
        return x_di, u_di

#ACCELERATING LINE PATH
def line_path_acc(start_position, end_position, t, T):
    a2 = 3/2/pow(T,2)
    a3 = -1/2/pow(T,3)
    s = a2*pow(t,2) + a3*pow(t,3)
    s_dot = 2*a2*t + 3*a3*pow(t,2)

    path_vec = end_position - start_position
    x_di = path_vec*s + start_position
    v_di = path_vec*s_dot
    theta_di = atan2((v_di.flatten())[1], (v_di.flatten())[0])
    thi_di = 0

    #construct x_di and u_di
    x_di = np.vstack((x_di, theta_di))
    u_di = np.array([[np.linalg.norm(v_di)], 
                     [thi_di]])
    
    return x_di, u_di

#DECELERATING LINE PATH
def line_path_dec(start_position, end_position, t, T):
    a1 = 3/2/T
    a3 = -1/2/pow(T,3)
    s = a1*t + a3*pow(t,3)
    s_dot = a1 + 3*a3*pow(t,2)

    path_vec = end_position-start_position
    x_di = path_vec*s + start_position
    v_di = path_vec*s_dot
    theta_di = atan2((v_di.flatten())[1], (v_di.flatten())[0]) + 2*pi
    thi_di = 0

    #construct x_di and u_di
    x_di = np.vstack((x_di, theta_di))
    u_di = np.array([[np.linalg.norm(v_di)], 
                     [thi_di]])

    return x_di, u_di

#DROPLET PATH
def droplet_path(start_position, t, T):
    R = droplet_radius #radius of the droplet circle

    first_path_vec = np.array([[R/sqrt(2)-start_position[0]], 
                               [R/sqrt(2)-start_position[1]]])
    scaling = np.linalg.norm(first_path_vec)
    T1 = T/(2 + pi*R/scaling)
    T2 = T1*pi*R/scaling
    T3 = T1
    
    if t <= T1:
        end_position = np.array([[R/sqrt(2)], 
                                 [R/sqrt(2)]])
        x_di, u_di = line_path_acc(start_position, end_position, t, T1)
        return x_di, u_di
    elif t <= (T1+T2):
        t2 = t-T1 #time for circular path
        x_position = R*cos(t2*3*pi/T2/2 - pi/4) + (start_position.flatten())[0]
        y_position = R*sin(t2*3*pi/T2/2 - pi/4) + (start_position.flatten())[1] + R*sqrt(2)
        v_x = -R*sin(t2*3*pi/T2/2 - pi/4)*3*pi/T2/2
        v_y = R*cos(t2*3*pi/T2/2 - pi/4)*3*pi/T2/2
        v_di = np.array([[v_x], 
                         [v_y]])
        theta_di = atan2(v_y, v_x)
        if theta_di < 0:
            theta_di = 2*pi + theta_di
        x_di = np.array([[x_position], 
                         [y_position], 
                         [theta_di]])
        thi_di = atan2(l1, R) #assume car goes counterclockwise
        u_di = np.array([[np.linalg.norm(v_di)], 
                         [thi_di]])
        return x_di, u_di
    else:
        t3 = t-T1-T2
        new_start_position = np.array([[-R/sqrt(2)], 
                                       [R/sqrt(2)]])
        x_di, u_di = line_path_dec(new_start_position, start_position, t3, T3)
        return x_di, u_di

#start_position = np.array([[0], [0]])
#print(droplet_path(start_position, 3, 20))







