#!/usr/bin/env python

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rospy
import time
import tf
import numpy as np


class Rover():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('controller')  

        self.rover_position = [0.0, 0.0, 0.0]
        self.desired_position = [15.0, 15.0, 0.0]

        self.pwm_cmd = Twist()
        self.pwm_cmd.linear.x = 1
        self.pwm_cmd.angular.z = 0.0

        self.state = 0
        self.ob_state = 0

        self.pwm_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.position_callback)
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
            self.ob_state = 0
            s = "nothing"
        elif self.regions["front"] > dist and self.regions["fright"] < dist and self.regions["fleft"] > dist:
            self.ob_state = 2
            s = "right"
        elif self.regions["front"] > dist and self.regions["fright"] > dist and self.regions["fleft"] < dist:
            self.ob_state = 0
            s = "left"
        elif self.regions["front"] < dist and self.regions["fright"] > dist and self.regions["fleft"] > dist:
            self.ob_state = 1
            s = "front"
        elif self.regions["front"] > dist and self.regions["fright"] < dist and self.regions["fleft"] < dist:
            self.ob_state = 0
            s = "right and left"
        elif self.regions["front"] < dist and self.regions["fright"] < dist and self.regions["fleft"] > dist:
            self.ob_state = 1
            s = "front and right"
        elif self.regions["front"] < dist and self.regions["fright"] < dist and self.regions["fleft"] < dist:
            self.ob_state = 1
            s = "all"
        elif self.regions["front"] < dist and self.regions["fright"] > dist and self.regions["fleft"] < dist:
            self.ob_state = 1
            s = "front and left"

        print(s)



    def position_callback(self, msg):
        self.rover_position[0] = msg.pose.pose.position.x
        self.rover_position[1] = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.rover_position[2] = tf.transformations.euler_from_quaternion(quaternion)[2]


    def fix_orientation(self):
        desired_angle = np.arctan2(self.desired_position[1] - self.rover_position[1], self.desired_position[0] - self.rover_position[0])
        error = desired_angle - self.rover_position[2]

        if abs(error) > np.pi / 180:
            self.pwm_cmd.angular.z = 0.4
        else:
            self.pwm_cmd.angular.z = 0
            self.state = 1
        self.pwm_cmd.linear.x = 0

        self.pwm_pub.publish(self.pwm_cmd)


    def go_straight(self):
        desired_angle = np.arctan2(self.desired_position[1] - self.rover_position[1], self.desired_position[0] - self.rover_position[0])
        angle_error = desired_angle - self.rover_position[2]

        if abs(angle_error) > np.pi / 180:
            self.state = 0
        else:           
            if abs(self.desired_position[0] - self.rover_position[0]) < 0.1 and abs(self.desired_position[1] - self.rover_position[1]) < 0.1: 
                self.state = 2
            else:
                self.pwm_cmd.angular.z = 0
                self.state = 1
                self.pwm_cmd.linear.x = 0.4
                self.pwm_pub.publish(self.pwm_cmd)


    def done(self):
        if abs(self.desired_position[0] - self.rover_position[0]) < 0.1 and abs(self.desired_position[1] - self.rover_position[1]) < 0.1: 
            self.pwm_cmd.angular.z = 0
            self.pwm_cmd.linear.x = 0
            self.pwm_pub.publish(self.pwm_cmd)


    def turn_left(self): # if obstacle found rotate so that wall is on the right side
	    self.pwm_cmd.angular.z = 0.4
	    self.pwm_cmd.linear.x = 0
	    self.pwm_pub.publish(self.pwm_cmd)


    def follow_wall(self): # move staight till wall is there
        self.pwm_cmd.angular.z = 0
        self.pwm_cmd.linear.x = 0.4
        self.pwm_pub.publish(self.pwm_cmd)


def main():
    
    car.obstacle_detect()
    if car.ob_state == 0: # if have to avoid obstacle do that or else go to goal
	    if car.state == 0:
	        car.fix_orientation()
	    elif car.state == 1:
	        car.go_straight()
	    elif car.state == 2:
	        car.done()
	    else:
	        print("unknown state")
    else:
	    if car.ob_state == 0:
	        car.state = 0
	    elif car.ob_state == 1:
	        car.turn_left()
	    elif car.ob_state == 2:
	        car.follow_wall()
	    else:
	        print("unknown state")
    print(car.rover_position)
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