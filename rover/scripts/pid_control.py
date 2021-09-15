#!/usr/bin/env python

# Importing the required libraries
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import rospy
import time
import tf
import numpy as np


# Class for attitude control
class Rover():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('controller')  # initializing ros node with name attitude_controller

        # This corresponds to current orientation of eDrone in quaternion format. This value will be updated each time in imu callback
        # [x,y,z,w]
        self.rover_position = [0.0, 0.0, 0.0]
        self.desired_position = [10.0, 0.0, 0.0]
        self.slope = self.desired_position[0] / self.desired_position[0]
        self.desired_position[2] = np.arctan2(self.desired_position[0], self.desired_position[0]) #desired angle,

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = Twist()
        self.pwm_cmd.linear.x = 1
        self.pwm_cmd.angular.z = 0.0

        self.t = time.time()
        self.current_time = 0

        # Kp, Kd and ki for [roll, pitch, yaw].
        self.Kp = [1, 0.6]
        self.Ki = [0.001, 0.0]
        self.Kd = [0.0, 0.0]

        self.error_sum = [0, 0]
        self.error_change = [0, 0]
        self.prev_error = [0, 0]
        self.max_values = 1
        self.min_values = 0

        # Publishing /edrone/pwm
        self.pwm_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscribing to /drone_command, /edrone/imu/data
        rospy.Subscriber('/odom', Odometry, self.position_callback)

    # Imu callback function
    def position_callback(self, msg):

        self.rover_position[0] = msg.pose.pose.position.x
        self.rover_position[1] = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.rover_position[2] = tf.transformations.euler_from_quaternion(quaternion)[2]


    # PID finction
    def pid(self):

        # Create variables to store errors and pid output.
        pid_output = [0, 0]

        dt = time.time() - self.t
        self.current_time += dt
        self.t = time.time()

        
        traj_x = self.current_time 
        if self.desired_position[0] < 0:
            traj_x = -traj_x
        traj_y = self.slope * traj_x


        if traj_x - self.rover_position[0] > 0:
            distance_error = np.sqrt((traj_x - self.rover_position[0]) ** 2 + (traj_y - self.rover_position[1]) ** 2)
        else:
            distance_error = -np.sqrt((traj_x - self.rover_position[0]) ** 2 + (traj_y - self.rover_position[1]) ** 2)
        self.error_sum[0] += distance_error
        self.error_change[0] = distance_error - self.prev_error[0]
        pid_output[0] = self.Kp[0] * distance_error + self.Kd[0] * self.error_change[0] + self.Ki[0] * self.error_sum[0]

        angle_error = self.desired_position[2] - self.rover_position[2]
        self.error_sum[1] += angle_error
        self.error_change[1] = angle_error - self.prev_error[1]
        pid_output[1] = self.Kp[1] * angle_error + self.Kd[1] * self.error_change[1] + self.Ki[1] * self.error_sum[1]
        

        self.prev_error = [distance_error, angle_error]

        # Controlling propeller speed using the PID outputs.

        if self.rover_position[0] > 10 or self.rover_position[1] > 10:
            self.pwm_cmd.linear.x = 0
            self.pwm_cmd.angular.z = 0
        else:
            self.pwm_cmd.linear.x = pid_output[0]
            # self.pwm_cmd.linear.x = 1
            # self.pwm_cmd.angular.z = 0
            self.pwm_cmd.angular.z = pid_output[1]



        # Limiting the output value between 0 and 1024
        # if self.pwm_cmd.prop1 > self.max_values[0]:
        #     self.pwm_cmd.prop1 = self.max_values[0]
        # if self.pwm_cmd.prop2 > self.max_values[1]:
        #     self.pwm_cmd.prop2 = self.max_values[1]
        # if self.pwm_cmd.prop3 > self.max_values[2]:
        #     self.pwm_cmd.prop3 = self.max_values[2]
        # if self.pwm_cmd.prop4 > self.max_values[3]:
        #     self.pwm_cmd.prop4 = self.max_values[3]

        # if self.pwm_cmd.prop1 < self.min_values[0]:
        #     self.pwm_cmd.prop1 = self.min_values[0]
        # if self.pwm_cmd.prop2 < self.min_values[1]:
        #     self.pwm_cmd.prop2 = self.min_values[1]
        # if self.pwm_cmd.prop3 < self.min_values[2]:
        #     self.pwm_cmd.prop3 = self.min_values[2]
        # if self.pwm_cmd.prop4 < self.min_values[3]:
        #     self.pwm_cmd.prop4 = self.min_values[3]

        # Publishing the propeller speeds
        self.pwm_pub.publish(self.pwm_cmd)

        print(distance_error, angle_error)

if __name__ == '__main__':

    car = Rover()
    r = rospy.Rate(20)  # specify rate in Hz based upon your desired PID sampling time.
    while not rospy.is_shutdown():
        try:
            car.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass


# in pid calculate desired angle using traj variables
# check v control
