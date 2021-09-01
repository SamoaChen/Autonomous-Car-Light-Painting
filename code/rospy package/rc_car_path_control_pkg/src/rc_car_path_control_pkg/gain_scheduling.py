#!/usr/bin/env python
from rc_car_path_control_pkg import l1, q_diag, r_diag
from control import lqr
from math import sin, cos, tan, pow
import numpy as np
import time

#-----FUNCTIONS
#LINEARIZED ERROR DYNAMICS
def generate_error_dynamics(x_di, u_di):
    theta = (x_di.flatten())[2];
    v = (u_di.flatten())[0];
    thi = (u_di.flatten())[1]

    #A MATRIX
    A = np.array([[0, 0, -v*sin(theta)],
                  [0, 0, v*cos(theta)],
                  [0, 0, 0]])

    #B MATRIX
    B = np.array([[cos(theta), 0],
                  [sin(theta), 0],
                  [tan(thi)/l1, v*pow((1/cos(thi)),2)/l1]])

    return A, B

#LQR CONTROLLER (average time 0.025s)
def lqr_cntrl(x, x_di, u_di):
    A, B = generate_error_dynamics(x_di, u_di)

    #tabulate Q and R matrix
    Q = np.diag(q_diag)
    R = np.diag(r_diag)

    #COMPUTE THE CONTROLLER GAIN
    K,_,_ = lqr(A, B, Q, R)

    #COMPUTE THE INPUT COMMAND
    u = u_di-np.dot(K,(x-x_di))
    #u = u_di
    
    return u

#old_time = time.time()
#x = np.array([[1], [2], [0]])
#x_di = np.array([[1.1], [2.3], [0.3]])
#u_di = np.array([[1], [0.5]])
#print(lqr_cntrl(x, x_di, u_di))
#print(time.time()-old_time)

