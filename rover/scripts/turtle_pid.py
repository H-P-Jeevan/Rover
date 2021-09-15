#!/usr/bin/env python

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import rospy
import time
import tf
import numpy as np


class Rover():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('controller')  

        self.rover_position = [0.0, 0.0, 0.0]

        self.t = time.time()

        self.pwm_cmd = Twist()
        self.pwm_cmd.linear.x = 1
        self.pwm_cmd.angular.z = 0.0

        self.v = 1
        self.w = 0

        self.pwm_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/turtle1/pose', Pose, self.position_callback)

    def position_callback(self, msg):
        self.rover_position[0] = msg.x - 6
        self.rover_position[1] = msg.y -4
        self.rover_position[2] = msg.theta

    def pid_control(self):
        dt = time.time() - self.t
        self.t = time.time()

        A = np.array([
            [ np.cos(self.rover_position[2]), -self.v * np.sin(self.rover_position[2])],
            [ np.sin(self.rover_position[2]), self.v * np.cos(self.rover_position[2])]
        ], np.float32)

        inv_A = np.linalg.inv(A)

        Kp = 1
        Kd = 2

        e = np.array([np.sin(self.t) - self.rover_position[0], np.cos(self.t) - self.rover_position[1]], np.float32).T
        de = np.array([np.cos(self.t), -np.sin(self.t)], np.float32).T - np.array([self.v * np.cos(self.rover_position[2]), self.v * np.sin(self.rover_position[2])]).T

        u = np.matmul(inv_A, np.array([[-np.sin(self.t)], [-np.cos(self.t)]]) + (Kp*e + Kd*de).reshape(2,1)) 

        a = u[0]
        self.w = u[1]

        self.v = self.v + a*dt

        self.pwm_cmd.linear.x = self.v
        self.pwm_cmd.angular.z = self.w

        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    car = Rover()
    r = rospy.Rate(20)  # specify rate in Hz based upon your desired PID sampling time.
    while not rospy.is_shutdown():
        try:
            car.pid_control()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass