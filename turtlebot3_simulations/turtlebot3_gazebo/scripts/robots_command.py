#!/usr/bin/env python
# coding: utf-8

import numpy as np
import socket, time

import tf

from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import Twist
import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from tf.transformations import *
from gazebo_msgs.msg import ModelStates

class Controller:
    # initialization method
    def __init__(self):

    	self.x_1, self.y_1, self.z_1 = 0, 0, 0
    	self.vx_1, self.vy_1, self.vz_1 = 0, 0, 0
    	self.q0_1, self.q1_1, self.q2_1, self.q3_1 = 0, 0, 0, 0

        self.x_2, self.y_2, self.z_2 = 0, 0, 0
        self.vx_2, self.vy_2, self.vz_2 = 0, 0, 0
        self.q0_2, self.q1_2, self.q2_2, self.q3_2 = 0, 0, 0, 0

        self.x_3, self.y_3, self.z_3 = 0, 0, 0
        self.vx_3, self.vy_3, self.vz_3 = 0, 0, 0
        self.q0_3, self.q1_3, self.q2_3, self.q3_3 = 0, 0, 0, 0

    	self.roll1, self.pitch1, self.yaw1 = 0,0,0 #current roll, pitch, yaw
        self.roll2, self.pitch2, self.yaw2 = 0,0,0 #current roll, pitch, yaw
        self.roll3, self.pitch3, self.yaw3 = 0,0,0 #current roll, pitch, yaw

        self.cmd_pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.cmd_pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
        self.cmd_pub3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)

        self.twist1 = Twist()
        self.twist2 = Twist()
        self.twist3 = Twist()

        self.robotpose1, self.robotpose2, self.robotpose3 = None,None, None
        self.robottwist1,self.robottwist2,self.robottwist3  = None,None, None


    def gazeboStateCb(self, msg):

        idx1 = msg.name.index('tb3_1')
        idx2 = msg.name.index('tb3_2')
        idx3 = msg.name.index('tb3_3')

        self.robotpose1 = msg.pose[idx1]
        self.robottwist1 = msg.twist[idx1]
        self.robotpose2 = msg.pose[idx2]
        self.robottwist2 = msg.twist[idx2]
        self.robotpose3 = msg.pose[idx3]
        self.robottwist3 = msg.twist[idx3]

        self.x_1 = self.robotpose1.position.x
        self.y_1 = self.robotpose1.position.y
        self.z_1 = self.robotpose1.position.z
        self.q0_1 = self.robotpose1.orientation.w
        self.q1_1 = self.robotpose1.orientation.x
        self.q2_1 = self.robotpose1.orientation.y
        self.q3_1 = self.robotpose1.orientation.z
        self.vx_1 = self.robottwist1.linear.x
        self.vy_1 = self.robottwist1.linear.y
        self.vz_1 = self.robottwist1.linear.z
        self.wx_1 = self.robottwist1.angular.x
        self.wy_1 = self.robottwist1.angular.y
        self.wz_1 = self.robottwist1.angular.z

        self.x_2 = self.robotpose2.position.x
        self.y_2 = self.robotpose2.position.y
        self.z_2 = self.robotpose2.position.z
        self.q0_2 = self.robotpose2.orientation.w
        self.q1_2 = self.robotpose2.orientation.x
        self.q2_2 = self.robotpose2.orientation.y
        self.q3_2 = self.robotpose2.orientation.z
        self.vx_2 = self.robottwist2.linear.x
        self.vy_2 = self.robottwist2.linear.y
        self.vz_2 = self.robottwist2.linear.z
        self.wx_2 = self.robottwist2.angular.x
        self.wy_2 = self.robottwist1.angular.y
        self.wz_2 = self.robottwist2.angular.z

        self.x_3 = self.robotpose3.position.x
        self.y_3 = self.robotpose3.position.y
        self.z_3 = self.robotpose3.position.z
        self.q0_3 = self.robotpose3.orientation.w
        self.q1_3 = self.robotpose3.orientation.x
        self.q2_3 = self.robotpose3.orientation.y
        self.q3_3 = self.robotpose3.orientation.z
        self.vx_3 = self.robottwist3.linear.x
        self.vy_3 = self.robottwist3.linear.y
        self.vz_3 = self.robottwist3.linear.z
        self.wx_3 = self.robottwist3.angular.x
        self.wy_3 = self.robottwist1.angular.y
        self.wz_3 = self.robottwist3.angular.z

        orientation_list1 = [self.q1_1, self.q2_1, self.q3_1, self.q0_1]
        (self.roll1, self.pitch1, self.yaw1) = euler_from_quaternion(orientation_list1)

        orientation_list2 = [self.q1_2, self.q2_2, self.q3_2, self.q0_2]
        (self.roll2, self.pitch2, self.yaw2) = euler_from_quaternion(orientation_list2)

        orientation_list3 = [self.q1_3, self.q2_3, self.q3_3, self.q0_3]
        (self.roll3, self.pitch3, self.yaw3) = euler_from_quaternion(orientation_list3)


    def cmd_velocity(self, vx1, wz1, vx2, wz2, vx3, wz3):

        self.twist1.linear.x = vx1
        self.twist1.linear.y = 0.0
        self.twist1.linear.z = 0.0
        self.twist1.angular.x = 0.0
        self.twist1.angular.y = 0.0 
        self.twist1.angular.z = wz1

        self.twist2.linear.x = vx2
        self.twist2.linear.y = 0.0
        self.twist2.linear.z = 0.0
        self.twist2.angular.x = 0.0
        self.twist2.angular.y = 0.0 
        self.twist2.angular.z = wz2

        self.twist3.linear.x = vx3
        self.twist3.linear.y = 0.0
        self.twist3.linear.z = 0.0
        self.twist3.angular.x = 0.0
        self.twist3.angular.y = 0.0 
        self.twist3.angular.z = wz3

    def pub_vel(self, vx1, wz1, vx2, wz2, vx3, wz3):
        self.cmd_velocity(vx1, wz1, vx2, wz2, vx3, wz3)
        self.cmd_pub1.publish(self.twist1)
        self.cmd_pub2.publish(self.twist2)
        self.cmd_pub3.publish(self.twist3)


def Tcp_connect( HostIp, Port ):
    global s
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HostIp, Port))
    return

def Tcp_Close( ):
   s.close()
   return 
 
def Tcp_Write_Array(myArray):
  myArrayString = ''
  for item in myArray:
    myArrayString = myArrayString + str(item) + "|"
  s.send((myArrayString).encode())  
  return

def Tcp_Read_Array():
  files = s.recv(1024)
  files = files.decode()
  myArray = files.split('|')
  return myArray


def main():
    rospy.init_node('setpoint_node', anonymous=True)
    cnt = Controller()
    rate = rospy.Rate(10.0)
    rospy.Subscriber('/gazebo/model_states', ModelStates, cnt.gazeboStateCb)

    Tcp_connect( '127.0.0.1', 17098) 

    while not rospy.is_shutdown():
        arr = [cnt.x_1, cnt.y_1, cnt.yaw1, 
                cnt.x_2, cnt.y_2, cnt.yaw2, 
                cnt.x_3, cnt.y_3, cnt.yaw3]

        
        Tcp_Write_Array(arr) 

        vel = Tcp_Read_Array()

        # print("----------")
        cnt.pub_vel(float(vel[0]), float(vel[1]),
                    float(vel[2]), float(vel[3]),
                    float(vel[4]), float(vel[5]) )


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
