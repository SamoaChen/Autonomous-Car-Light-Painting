#!/usr/bin/env python

import rospy
import serial
from rc_car_path_control_pkg.msg import Car_actuation
from rc_car_path_control_pkg import start_servo, encode_servo_angle, end_servo, start_uno_serial, conv_encode_velocity, servo_pin, steering_to_servo, thi_shrink_coef, v_shrink_coef

#-----DEFINE GLOBAL OBJECT
my_serial = start_uno_serial()
my_servo = start_servo()

#-----FUNCTIONS
#ACTUATION EXECUTION
def actuation_execution(car_velocity_string, dutyCycle):
    #actuate rear wheels
    my_serial.write(b'r')
    my_serial.write(car_velocity_string)
    #actuate servo
    my_servo.set_servo_pulsewidth(servo_pin, dutyCycle)

#-----ACTUATOR CLASS
class Actuator(object):
    """actuator for servo and rear wheels"""
    def __init__(self):
        self.actuation_sub = rospy.Subscriber('actuation_command', Car_actuation, self.actuation_sub_callback, queue_size=1)
        rospy.on_shutdown(self.clean_up)

        #initiate car_velocity_string servo_angle dutyCycle
        self.car_velocity_string =  ''
        self.servo_angle = 0.0
        self.dutyCycle = 0

    def actuation_sub_callback(self, actuation_read):
        self.car_velocity_string = conv_encode_velocity(actuation_read.v/v_shrink_coef)
        self.servo_angle = steering_to_servo(actuation_read.thi/thi_shrink_coef)
        self.dutyCycle = encode_servo_angle(self.servo_angle)
        actuation_execution(self.car_velocity_string, self.dutyCycle)

    def clean_up(self):
        my_serial.close()
        end_servo()
    
if __name__ == "__main__":
    rospy.init_node('rc_car_actuation_node', anonymous=True)
    actuator = Actuator()
    rospy.spin()
