#!/usr/bin/env python
import time
import rospy
import numpy as np
from rc_car_path_control_pkg import generate_path, lqr_cntrl
from rc_car_path_control_pkg.msg import Car_state, Car_actuation, Car_trajectory_command

#-----CREATE CONTROLLER CLASS
class Controller(object):
    """controller class for rc car path control"""
    def __init__(self):
        self.command_sub = rospy.Subscriber('trajectory_command', Car_trajectory_command, self.command_sub_callback, queue_size=1)
        self.state_sub = rospy.Subscriber('estimated_state', Car_state, self.state_sub_callback, queue_size=1)
        self.actuation_pub = rospy.Publisher('actuation_command', Car_actuation, queue_size=1)
        self.rate = rospy.Rate(100)


        #initialize x u x_di u_di t_current T t_old command car_state car_actuation
        self.x = np.array([[0.0],
                           [0.0],
                           [0.0]])
        self.u = np.array([[0.0],
                           [0.0]])
        self.x_di = np.array([[0.0],
                              [0.0],
                              [0.0]])
        self.u_di = np.array([[0.0],
                              [0.0]])
        self.t_current = 0
        self.T_command = 0
        self.t_old = 0
        self.command = 'stop'
        self.car_state = Car_state()
        self.car_actuation = Car_actuation()

    def command_sub_callback(self, command_read):
        self.command = command_read.command
        if self.command == 'droplet':
            self.t_old = time.time()
            self.T_command = command_read.T
    def state_sub_callback(self, state_read):
        self.car_state = state_read
        #update x
        self.x[0] = self.car_state.x
        self.x[1] = self.car_state.y
        self.x[2] = self.car_state.theta

        #trajectory stop condition
        if self.t_current > self.T_command:
            self.command = 'stop'
            self.t_current = 0
        
        #generate x_di and u_di
        if self.command == 'initialize':
            self.car_actuation.v = 0.0
            self.car_actuation.thi = 0.0
            time.sleep(0.01) #delay the communication for arduino to catch upl
            self.actuation_pub.publish(self.car_actuation)
        elif self.command == 'droplet':
            self.t_current = time.time() - self.t_old
            self.x_di, self.u_di = generate_path(t=self.t_current, T=self.T_command, activation_no=3)
            #lqr controller
            self.u = lqr_cntrl(self.x, self.x_di, self.u_di)
            #update car actuation
            self.car_actuation.v = self.u[0]
            self.car_actuation.thi = self.u[1]
            #pace the updating speed
            self.rate.sleep()
            self.actuation_pub.publish(self.car_actuation)
        elif self.command == 'stop':
            self.car_actuation.v = 0.0
            self.car_actuation.thi = 0.0
            time.sleep(0.01) #delay the communication for arduino to catch up
            self.actuation_pub.publish(self.car_actuation)


if __name__ == "__main__":
    rospy.init_node('controller_node', anonymous=True)
    controller = Controller()
    rospy.spin()
