#!/usr/bin/env python

import serial
from rc_car_path_control_pkg import IMU_port, IMU_baudrate

#-----FUNCTIONS
#START SENSOR SERIAL
def start_IMU_serial():
    ser = serial.Serial(IMU_port, IMU_baudrate, timeout=0.5)
    
    return ser

#DECODE COURSE ANGLE
def decode_course_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180

    angle_x = (rxh << 8 | rxl) / 32768 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >=k_angle:
        angle_z-= 2 * k_angle

    return angle_z

