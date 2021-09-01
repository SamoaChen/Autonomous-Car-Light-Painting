#!/usr/bin/env python
import numpy as np
from math import pi, pow

#-----GEOMETRIC PARAMETERS
l1 = 0.14325           #(m)length from steering to rear
l2 = 0.049             #(m)length from center of steering to the right wheel pivot
la = 0.024459          #(m)length length of wheel pivot arm
ls = 0.0196            #(m)length of servo arm
theta_c = -99.6*pi/180 #(rad)wheel shaft structural angle
l_lever = 0.0565       #(m)lever arm length
d_wheel = 0.065        #(m)radius of car wheel

#-----SERIAL PARAMETERS
arduino_port = '/dev/ttyUSB1' #UNO's serial port address
arduino_baudrate = 9600       #UNO's serial baudrate
IMU_port = '/dev/ttyUSB0'     #IMU's serial port address
IMU_baudrate = 115200         #IMU's serial baudrate

#-----SERVO PARAMETER
servo_pin = 18                #servo pwm pin following GPIO convention
servo_angle_offset = 0        #servo rotor angle offset
angle_lowerbound = 33         #minimum angle command
angle_higherbound = 135       #maximum angle command

#-----EKF PARAMETERS
q = np.array([5.6e-4, 3.3e-5, 4.8e-3]) #standard deviation for state noise
r = 0.001*pi/180                           #standard deviation for angle sensor noise
H = np.array([[0, 0, 1]])                #the measurement matrix C

#-----TRAJECTORY PARAMETERS
trajectory_selection = 3
droplet_radius = 1 #(m) radius for the droplet trajectory
default_T = 10 #(s)default time to complete trajectory

#-----LQR PARAMETERS
q_diag = np.array([800, 800, 50])
r_diag = np.array([50, 50])

#-----CONTROLLER PARAMETERS
#initialize a default start position
default_start_position = np.array([[0],
                                   [0]])
#-----STEERING PARAMETRS
thi_shrink_coef = 1 #0.45
v_shrink_coef = 6.55/5
