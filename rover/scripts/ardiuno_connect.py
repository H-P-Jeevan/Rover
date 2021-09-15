#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import String
import rospy
import time

class Rover():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('communicate')  

        self.velocity = 0.0
        self.angular_velocity = 0.0

        self.pwm_pub = rospy.Publisher('/chatter', String, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)


    def velocity_callback(self, msg):
        self.velocity = int(msg.linear.x*1000) * 0.001
        self.angular_velocity = int(msg.angular.z * 1000) * 0.001 # decimal places


if __name__ == '__main__':

    car = Rover()
    r = rospy.Rate(20)  # specify rate in Hz based upon your desired PID sampling time.
    while not rospy.is_shutdown():
        try:
            car.pwm_pub.publish(str(car.angular_velocity) + "  " + str(car.velocity))
            print(str(car.angular_velocity) + "  " + str(car.velocity))
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass