#!/usr/bin/env python

import rospy
from math import pi
from rc_car_path_control_pkg.msg import Car_course
from rc_car_path_control_pkg import start_IMU_serial, decode_course_angle


#-----GLOBAL SERIAL OBJECT
IMU_serial = start_IMU_serial()

#-----IMU CLASS
class IMU(object):
    """measure course angle from IMU reading"""
    def __init__(self):
        self.course_angle_pub = rospy.Publisher('course_angle', Car_course, queue_size=1)
        self.AngleData = [0.0]*8
        self.FrameState = 0
        self.Bytenum = 0
        self.CheckSum = 0
        self.car_course = Car_course()
        self.start_angle = 45
        self.theta = self.start_angle
        self.theta_old = 0.0
        self.theta_new = 0.0
        self.read_success = False
        self.initialization()
        rospy.on_shutdown(self.clean_up)
    
    def clean_up(self):
        IMU_serial.close()
    
    def publish_course_angle(self):
        while True:
            if IMU_serial.inWaiting():
                self.read_course_angle(IMU_serial.read(33))
                self.theta_update()
                self.car_course.theta = round(self.theta*pi/180, 4)
                self.course_angle_pub.publish(self.car_course)

    def theta_update(self):
        if (self.theta_old*self.theta_new) >= 0:
            self.theta = self.theta + (self.theta_new - self.theta_old)
        elif self.theta_old > 90:
            self.theta = self.theta + (360 + self.theta_new - self.theta_old)
        elif self.theta_old > 0:
            self.theta = self.theta -(self.theta_old - self.theta_new)
        elif self.theta_old < -90:
            self.theta = self.theta - (360 + self.theta_old - self.theta_new)
        elif self.theta_old < 0:
            self.theta = self.theta + (self.theta_new - self.theta_old)
        self.theta_old = self.theta_new

    def initialization(self):
        while not self.read_success:
            if IMU_serial.inWaiting():
                self.read_course_angle(IMU_serial.read(33))
        self.theta_old = self.theta_new

    def read_course_angle(self, inputdata):
        for data in inputdata:
            if self.FrameState==0:
                if data==0x55 and self.Bytenum==0:
                    self.CheckSum=data
                    self.Bytenum=1
                    continue
                elif data==0x53 and self.Bytenum==1:
                    self.CheckSum+=data
                    self.FrameState=3
                    self.Bytenum=2
            elif self.FrameState==3: # angle
                if self.Bytenum<10:
                    self.AngleData[self.Bytenum-2]=data
                    self.CheckSum+=data
                    self.Bytenum+=1
                else:
                    if data == (self.CheckSum&0xff):
                        pass
                        self.theta_new = round(decode_course_angle(self.AngleData), 2)
                        self.read_success = True

                    self.CheckSum=0
                    self.Bytenum=0
                    self.FrameState=0
      

if __name__ == "__main__":
    rospy.init_node('course_angle_node', anonymous=True)
    imu = IMU()
    imu.publish_course_angle()
