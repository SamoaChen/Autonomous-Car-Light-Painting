#!/usr/bin/env python
import rospy
import time
import numpy as np
from rc_car_path_control_pkg import EKF, q
from rc_car_path_control_pkg.msg import Car_actuation, Car_state, Car_course


#-----CREATE OBSERVER CLASS
class Observer(object):
    """observer class for rc car state update"""
    def __init__(self):
        self.course_sub = rospy.Subscriber('course_angle', Car_course, self.course_sub_callback, queue_size=1)
        self.actuation_sub = rospy.Subscriber('actuation_command', Car_actuation, self.actuation_sub_callback, queue_size = 1)
        self.state_pub = rospy.Publisher('estimated_state', Car_state, queue_size = 1)
        
        #initialize car_state car_course car_actuation x_pos p_pos t_old u_old dt
        self.car_state = Car_state()
        self.car_course = Car_course()
        self.car_actuation = Car_actuation() #elements are initialized to be 0s
        self.x_pos = np.array([[0.0],
                               [0.0],
                               [0.0]])
        self.p_pos =  np.diag(np.square(q))
        self.t_old = time.time()
        self.u_old = np.array([[0.0],
                               [0.0]])

    def course_sub_callback(self, car_course_read):
        self.car_course = car_course_read
        t_new = time.time()
        dt = t_new - self.t_old
        #update t_old 
        self.t_old = t_new
        #update x_pos p_pos
        self.x_pos, self.p_pos = EKF(dt, self.x_pos, self.p_pos, self.u_old, self.car_course.theta)
        #update u_old
        self.u_old[0] = self.car_actuation.v
        self.u_old[1] = self.car_actuation.thi
        #update car_state 
        self.car_state.x = self.x_pos[0]
        self.car_state.y = self.x_pos[1]
        self.car_state.theta = self.x_pos[2]
        #publish car_state
        self.state_pub.publish(self.car_state)

    def actuation_sub_callback(self, car_actuation_read):
        self.car_actuation = car_actuation_read
        t_new = time.time()
        dt = t_new - self.t_old
        #update t_old
        self.t_old = t_new
        #update x_pos p_pos
        self.x_pos, self.p_pos = EKF(dt, self.x_pos, self.p_pos, self.u_old, self.car_course.theta)
        #update u_old
        self.u_old[0] = self.car_actuation.v
        self.u_old[1] = self.car_actuation.thi
        #update car_state 
        self.car_state.x = self.x_pos[0]
        self.car_state.y = self.x_pos[1]
        self.car_state.theta = self.x_pos[2]
        #publish car_state
        self.state_pub.publish(self.car_state)

if __name__ == "__main__":
    rospy.init_node('state_observer_node', anonymous=True)
    observer = Observer()
    rospy.spin()


