#!/usr/bin/env python
import os
import time
import serial
import pigpio
from math import pi
from rc_car_path_control_pkg import arduino_port, arduino_baudrate, servo_pin, servo_angle_offset, angle_lowerbound, angle_higherbound, d_wheel

#-----FUNCTIONS
#START ARDUINO SERIAL COMMUNICATION
def start_uno_serial():
    ser = serial.Serial(arduino_port, baudrate=arduino_baudrate, timeout=1)
    print('Please wait 1 second for serial communication to initiate...')
    time.sleep(1)
    print('Arduino serial is ready!')
    ser.reset_input_buffer()
    
    return ser

#CONVERT CAR VELOCITY AND ENCODE
def conv_encode_velocity(v):
    #limit velocity command within bound
    if v > 1.2:
        v = 1.2
    elif v < -1.2:
        v = -1.2
    C = pi*d_wheel #circumference of the wheel
    rps = v/C #revolution per second
    cps = int(rps*780) #encoder pulse per second
    car_velocity_string = str.encode(str(cps)+'*')
    
    return car_velocity_string

#ser.close()

#START SERVO 
def start_servo():
    os.system('sudo pigpiod')
    print('Please wait 1 second for pigpiod to function...')
    time.sleep(1)
    my_servo = pigpio.pi()
    print('Servo is ready!')
    
    return my_servo

#ENCODE SERVO angle
def encode_servo_angle(servo_angle):
    #convert servo_angle to degree
    servo_angle = servo_angle*180/pi
    #convert to actual servo command
    servo_angle = servo_angle + servo_angle_offset
    
    #limit servo angle within bound
    if servo_angle > 135:
        servo_angle = 135
    elif servo_angle < 33:
        servo_angle = 33
    dutyCycle = int(servo_angle*2000/180 + 500)

    return dutyCycle
   
#my_servo.set_servo_pulsewidth(servo_pin, dutyCycle)

#DEACTIVATE SERVO
def end_servo():
    os.system('sudo killall pigpiod')


