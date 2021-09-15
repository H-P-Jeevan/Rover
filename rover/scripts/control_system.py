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
        self.desired_position = [5.0, -6.0]
        
        self.t = time.time()

        self.pwm_cmd = Twist()
        self.pwm_cmd.linear.x = 1
        self.pwm_cmd.angular.z = 0.0

        self.v = 1
        self.w = 0

        self.max_v = 1
        self.max_w = 1

        self.Kp = 1
        self.Kd = 2

        self.pwm_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.position_callback)


    def position_callback(self, msg):
        self.rover_position[0] = msg.pose.pose.position.x - self.desired_position[0]
        self.rover_position[1] = msg.pose.pose.position.y - self.desired_position[1]
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.rover_position[2] = tf.transformations.euler_from_quaternion(quaternion)[2]


    def pid_control(self):
        dt = time.time() - self.t
        self.t = time.time()


        e = np.array([np.sin(self.t) - self.rover_position[0], np.cos(self.t) - self.rover_position[1]], np.float32).T # [xr - x; yr - y]
        de = np.array([np.cos(self.t), -np.sin(self.t)], np.float32).T - self.v * np.array([np.cos(self.rover_position[2]), np.sin(self.rover_position[2])]).T # [x.r; y.r] - [x.; y.]  


        A = np.array([ # A = rotation matrix
            [ np.cos(self.rover_position[2]), -self.v * np.sin(self.rover_position[2])],
            [ np.sin(self.rover_position[2]), self.v * np.cos(self.rover_position[2])]
        ], np.float32)

        inv_A = np.linalg.inv(A)        

        u = np.matmul(inv_A, np.array([[-np.sin(self.t)], [-np.cos(self.t)]]) + (self.Kp*e + self.Kd*de).reshape(2,1)) # u = A-1 * ([x..r; y..r] + pid)
        a = u[0]
        
        self.w = u[1]
        self.v = self.v + a*dt # v = u +at

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
