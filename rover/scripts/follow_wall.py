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

        rospy.init_node('controller')  

        self.pwm_cmd = Twist()
        self.pwm_cmd.linear.x = 1
        self.pwm_cmd.angular.z = 0.0

        self.state = 0

        self.pwm_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/m2wr/laser/scan', LaserScan, self.laser_callback)

        self.regions = {
        "right" : 10,
        "fright" : 10,
        "front" : 10,
        "fleft" : 10,
        "left" : 10,
        }

    def laser_callback(self, msg):
        self.regions = {
            "right" : min(min(msg.ranges[0:143]), 10),
            "fright" : min(min(msg.ranges[144:287]), 10),
            "front" : min(min(msg.ranges[288:431]), 10),
            "fleft" : min(min(msg.ranges[432:575]), 10),
            "left" : min(min(msg.ranges[576:719]), 10)
        }

    def obstacle_detect(self):
        dist = 1.5
        if self.regions["front"] > dist and self.regions["fright"] > dist and self.regions["fleft"] > dist:
            self.state = 0
            s = "nothing"
        elif self.regions["front"] > dist and self.regions["fright"] < dist and self.regions["fleft"] > dist:
            self.state = 2
            s = "right"
        elif self.regions["front"] > dist and self.regions["fright"] > dist and self.regions["fleft"] < dist:
            self.state = 0
            s = "left"
        elif self.regions["front"] < dist and self.regions["fright"] > dist and self.regions["fleft"] > dist:
            self.state = 1
            s = "front"
        elif self.regions["front"] > dist and self.regions["fright"] < dist and self.regions["fleft"] < dist:
            self.state = 0
            s = "right and left"
        elif self.regions["front"] < dist and self.regions["fright"] < dist and self.regions["fleft"] > dist:
            self.state = 1
            s = "front and right"
        elif self.regions["front"] < dist and self.regions["fright"] < dist and self.regions["fleft"] < dist:
            self.state = 1
            s = "all"
        elif self.regions["front"] < dist and self.regions["fright"] > dist and self.regions["fleft"] < dist:
            self.state = 1
            s = "front and left"

        print(s)


    def find_wall(self):
        self.pwm_cmd.angular.z = -0.2
        self.pwm_cmd.linear.x = 0.2
        self.pwm_pub.publish(self.pwm_cmd)


    def turn_left(self):
	    self.pwm_cmd.angular.z = 0.4
	    self.pwm_cmd.linear.x = 0
	    self.pwm_pub.publish(self.pwm_cmd)


    def follow_wall(self):
        self.pwm_cmd.angular.z = 0
        self.pwm_cmd.linear.x = 0.4
        self.pwm_pub.publish(self.pwm_cmd)


def main():
    
    car.obstacle_detect()
    if car.state == 0:
        car.find_wall()
    elif car.state == 1:
        car.turn_left()
    elif car.state == 2:
        car.follow_wall()
    else:
        print("unknown state")
    print(car.state)


if __name__ == '__main__':

    car = Rover()
    r = rospy.Rate(20)  # specify rate in Hz based upon your desired PID sampling time.
    while not rospy.is_shutdown():
        try:
            main()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass