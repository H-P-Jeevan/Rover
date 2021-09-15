#!/usr/bin/env python

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import rospy
import time
import tf
import numpy as np


class Rover():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('obstacle_avoid')  

        self.regions = {
        "right" : 10,
        "fright" : 10,
        "front" : 10,
        "fleft" : 10,
        "left" : 10,
        }

        self.pwm_cmd = Twist()
        self.pwm_cmd.linear.x = 0.6
        self.pwm_cmd.angular.z = 0.6

        self.state = 0

        self.pwm_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/m2wr/laser/scan', LaserScan, self.laser_callback)

    def laser_callback(self, msg):
        self.regions = {
            "right" : min(min(msg.ranges[0:143]), 10),
            "fright" : min(min(msg.ranges[144:287]), 10),
            "front" : min(min(msg.ranges[288:431]), 10),
            "fleft" : min(min(msg.ranges[432:575]), 10),
            "left" : min(min(msg.ranges[576:719]), 10)
        }

    def obstacle_avoid(self):
        dist = 1
        if self.regions["front"] > dist and self.regions["fright"] > dist and self.regions["fleft"] > dist:
            self.pwm_cmd.linear.x = 0.6
            self.pwm_cmd.angular.z = 0
            s = "nothing"
        elif self.regions["front"] > dist and self.regions["fright"] < dist and self.regions["fleft"] > dist:
            self.pwm_cmd.linear.x = 0.1
            self.pwm_cmd.angular.z = 0.3
            s = "right"
        elif self.regions["front"] > dist and self.regions["fright"] > dist and self.regions["fleft"] < dist:
            self.pwm_cmd.linear.x = 0.1
            self.pwm_cmd.angular.z = -0.3
            s = "left"
        elif self.regions["front"] < dist and self.regions["fright"] > dist and self.regions["fleft"] > dist:
            self.pwm_cmd.linear.x = 0
            self.pwm_cmd.angular.z = 0.3
            s = "front"
        elif self.regions["front"] > dist and self.regions["fright"] < dist and self.regions["fleft"] < dist:
            self.pwm_cmd.linear.x = 0
            self.pwm_cmd.angular.z = 0.6
            s = "right and left"
        elif self.regions["front"] < dist and self.regions["fright"] < dist and self.regions["fleft"] > dist:
            self.pwm_cmd.linear.x = 0
            self.pwm_cmd.angular.z = 0.6
            s = "front and right"
        elif self.regions["front"] < dist and self.regions["fright"] < dist and self.regions["fleft"] < dist:
            self.pwm_cmd.linear.x = 0
            self.pwm_cmd.angular.z = 0.6
            s = "all"
        elif self.regions["front"] < dist and self.regions["fright"] > dist and self.regions["fleft"] < dist:
            self.pwm_cmd.linear.x = 0
            self.pwm_cmd.angular.z = -0.6
            s = "front and left"

        print(self.pwm_cmd.linear.x, self.pwm_cmd.angular.y)
        print(s)

        self.pwm_pub.publish(self.pwm_cmd)
 

if __name__ == '__main__':

    car = Rover()
    r = rospy.Rate(20)  # specify rate in Hz based upon your desired PID sampling time.
    while not rospy.is_shutdown():
        try:
            car.obstacle_avoid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass