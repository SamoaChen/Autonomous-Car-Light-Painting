#!/usr/bin/env python
import time
import rospy
from math import pi

from rc_car_path_control_pkg.msg import Car_actuation
car_actuation = Car_actuation()
car_actuation.thi = pi/6

def actuation_publisher():
    pub = rospy.Publisher('actuation_command', Car_actuation, queue_size=1)
    rospy.init_node('actuation_send_node', anonymous=True)
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        if car_actuation.v < 0.3:
            pub.publish(car_actuation)
            car_actuation.v += 0.01
        rate.sleep()

if __name__ == '__main__':
    actuation_publisher()

