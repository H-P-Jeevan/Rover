#!/usr/bin/env python

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import rospy
import time
import tf
import numpy as np


class Rover():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('controller')  

        self.rover_position = [0.0, 0.0, 0.0]
        self.desired_position = [15.0, -16.0, 0.0]
        self.slope = self.desired_position[0] / self.desired_position[0]
        self.desired_position[2] = np.arctan2(self.desired_position[0], self.desired_position[0]) 

        self.pwm_cmd = Twist()
        self.pwm_cmd.linear.x = 1
        self.pwm_cmd.angular.z = 0.0

        self.state = 0

        self.pwm_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.position_callback)


    def position_callback(self, msg):
        self.rover_position[0] = msg.pose.pose.position.x
        self.rover_position[1] = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.rover_position[2] = tf.transformations.euler_from_quaternion(quaternion)[2]


    def fix_orientation(self):
        desired_angle = np.arctan2(self.desired_position[1], self.desired_position[0])
        error = desired_angle - self.rover_position[2]

        if abs(error) > np.pi / 90:
            self.pwm_cmd.angular.z = 0.7
        else:
            self.pwm_cmd.angular.z = 0
            self.state = 1
        self.pwm_cmd.linear.x = 0

        self.pwm_pub.publish(self.pwm_cmd)


    def go_straight(self):
        desired_angle = np.arctan2(self.desired_position[1], self.desired_position[0])
        angle_error = desired_angle - self.rover_position[2]

        if abs(angle_error) > np.pi / 90:
            self.state = 0
        else:           
            if abs(self.desired_position[0] - self.rover_position[0]) < 0.1 and abs(self.desired_position[1] - self.rover_position[1]) < 0.1: 
                self.state = 2
            else:
                self.pwm_cmd.angular.z = 0
                self.state = 1
                self.pwm_cmd.linear.x = 1
                self.pwm_pub.publish(self.pwm_cmd)


    def done(self):
        if abs(self.desired_position[0] - self.rover_position[0]) < 0.1 and abs(self.desired_position[1] - self.rover_position[1]) < 0.1: 
            self.pwm_cmd.angular.z = 0
            self.pwm_cmd.linear.x = 0
            self.pwm_pub.publish(self.pwm_cmd)


def main():
    if car.state == 0:
        car.fix_orientation()
    elif car.state == 1:
        car.go_straight()
    elif car.state == 2:
        car.done()
    else:
        print("unknown state")
    print(car.state)


if __name__ == '__main__':

    car = Rover()
    r = rospy.Rate(20)  # specify rate in Hz based upon your desired PID sampling time.
    while not rospy.is_shutdown():
        try:
            main()
            # car.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass