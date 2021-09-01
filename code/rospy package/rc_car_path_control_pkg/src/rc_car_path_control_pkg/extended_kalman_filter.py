#!/usr/bin/env python
from math import sin, cos, tan, pow, pi
from rc_car_path_control_pkg import l1, q, r, H
import numpy as np


#-----DEFINE GLOBAL PARAMETERS FOR EKF
Q = np.diag(np.square(q)) #covariance matrix for state noise
R = pow(r,2) #covariance for angle sensor noise

#-----FUNCTIONS
#TRANSITION JACOBIAN
def transition_jacob(dt, x_pos, u_old):
    #state variables and actuation signals
    theta = (x_pos.flatten())[2]
    v = (u_old.flatten())[0]

    F = np.array([[1, 0, -v*sin(theta)*dt],
                  [0, 1, v*cos(theta)*dt],
                  [0, 0, 1]])
    return F

#SYSTEM DIFFERENCE EQUATION UPDATE
def sys_difference_update(dt, x_pos, u_old):
    #state variables and actuation signals
    x = (x_pos.flatten())[0]
    y = (x_pos.flatten())[1]
    theta = (x_pos.flatten())[2]
    v = (u_old.flatten())[0]
    thi = (u_old.flatten())[1]

    x_pri = np.array([[x + v*cos(theta)*dt],
                      [y + v*sin(theta)*dt],
                      [theta + v*tan(thi)*dt/l1]])
    return x_pri

#EXTENDED KALMAN FILTER
def EKF(dt, x_pos, p_pos, u_old, theta):
    #prediction
    x_pri = sys_difference_update(dt, x_pos, u_old)
    F = transition_jacob(dt, x_pos, u_old)
    p_pri = np.dot(np.dot(F, p_pos), np.transpose(F)) + Q

    #update
    y_hat = theta - np.dot(H, x_pri) #difference between the predicted and measured y values
    S = np.dot(np.dot(H, p_pri), np.transpose(H)) + R

    K_f = np.dot(p_pri, np.transpose(H))/S #because S is scalar, times its inverse equals to divide by it
    x_pos = x_pri + K_f*y_hat #here y_hat is scalar, thus use * to multiply
    p_pos =np.dot((np.identity(3) - np.dot(K_f, H)), p_pri)
    #print(p_pos)
    
    return x_pos, p_pos


#dt = 0.005
#x_pos = np.array([[2], [1], [0.5]])
#p_pos = np.array([[2, 0, 0],
#                  [0, 2, 0],
#                  [0, 0, 2]])
#u_old = np.array([[1], [0.8]])
#theta = 0.6
#print(EKF(dt, x_pos, p_pos, u_old, theta))


