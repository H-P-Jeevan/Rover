# Rover

In this project we deal with the design and control of a rover that can move to desired positions. The rover consists of 3 wheels and a base_link. It consists of laser sensors that are used to detect any surrounding obstacles and map the surroundings using the gmapping package. The robot can be controlled by a non linear controller. The robot can be controlled using a website also. Different pyhton programs can are written to acheive different tasks. 

---

### Table of Contents

- [Installation](#installation)
- [Technologies](#technologies)
- [How To Use](#how-to-use)
- [References](#references)
- [Author Info](#author-info)

---

## Installation

- Use these commands to install ROS Melodic on Ubuntu 18.04: 
    > sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  
    > sudo apt install curl  
    > curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -  
    > sudo apt update  
    > sudo apt install ros-melodic-desktop-full

- Install git
    > apt-get install git

- Creating catkin workspace: 
    > source /opt/ros/melodic/setup.bash  
    > mkdir -p ~/catkin_ws/src  
    > cd ~/catkin_ws/  
    > catkin_make  
    > source devel/setup.bash
 
 - Cloning the github repository
    > cd ~/catkin_ws/src/  
    > git clone https://github.com/H-P-Jeevan/Rover.git

 - Installing packages
    > sudo apt install ros-melodic-slam-gmapping  
    > sudo apt-get install ros-melodic-rosbridge-server
 
---

## Technologies

- ROS
- Gmapping
- Arduino
- HTML, CSS, Javascript

---

## How To Use

#### Running gazebo simulation
  > roslaunch rover gazebo.launch

#### Running differnt controllers
 - Run non linear control system 
    > rosrun rover control_system.py
 - Run follow wall
    > rosrun rover follow_wall.py
 - Run obstacle avoidance
    > rosrun rover obstacle_avoid.py
 - Run go to goal
    > rosrun rover simple_control.py 


#### ROS and arduino
 - Terminal 1
    > sudo chmod a+rw /dev/ttyACM0  
    > rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=57600
 - Terminal 2
    > rosrun rover ardiuno_connect.py

#### ROS and SLAM
 - Terminal 1
    > roslaunch rover ggmapping.launch
 - Terminal 2
    > rviz

---

## References
Gazebo plugins
   > http://gazebosim.org/tutorials?tut=ros_gzplugins

Gazebo joints
   > http://wiki.ros.org/urdf/XML/joint

ros and web communication
   > https://msadowski.github.io/ros-web-tutorial-pt1/

---

## Author Info

- H P Jeevan https://www.linkedin.com/in/h-p-jeevan-08607a1a8
