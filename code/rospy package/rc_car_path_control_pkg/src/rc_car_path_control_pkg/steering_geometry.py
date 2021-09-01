#!/usr/bin/env python
from rc_car_path_control_pkg import l1, l2, la, ls, theta_c, l_lever #import car's geometric parameters
from math import sin, cos, asin, acos, tan, atan2, pow, sqrt

#-----FUNCTIONS
#STEERING AND SERVO ANGLE CONVERSION
def steering_to_servo(thi):
    theta = steering_to_wheel(thi)
    theta_s = wheel_to_servo(theta)

    return theta_s

#STEERING AND WHEEL ANGLE CONVERSION
def steering_to_wheel(thi):
    #the steering geometry calculation has its limited range given plotted function from -0.9076 to 1.1868
    if thi < -0.9076:
        thi = -0.9076
    elif thi > 1.1868:
        thi = 1.1868
    #seperate cases for calculating theta
    if thi == 0:
        theta = 0
    elif thi > 0:
        theta = atan2(l1,l1/tan(thi)+l2)
    elif thi < 0:
        theta = -atan2(l1,l1/tan(-thi)-l2)
    return theta

#WHEEL AND SERVO ANGLE CONVERSION
def wheel_to_servo(theta):
    #wheel shaft end coordinate
    x_p = la*cos(theta_c+theta)
    y_p = la*sin(theta_c+theta)
    z_p = 0
    x_s = -0.059
    y_s = -0.02
    z_s = 0.003

    #use sine rule to calculate servo angle
    a = ls
    b = sqrt(pow(x_p-x_s,2)+pow(z_p-z_s,2))
    c = l_lever*sqrt(1-pow((y_p-y_s)/l_lever,2))
    #print((pow(a,2)+pow(b,2)-pow(c,2))/(2*a*b))

    theta_s = acos((pow(a,2)+pow(b,2)-pow(c,2))/(2*a*b))

    #get rid of the bottom offset angle 
    theta_s_offset = asin(z_s/b)
    theta_s = theta_s-theta_s_offset

    return theta_s
#print(steering_to_servo(3.14159/3))
