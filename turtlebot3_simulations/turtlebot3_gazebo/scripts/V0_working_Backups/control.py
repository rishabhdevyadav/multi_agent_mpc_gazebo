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

    	self.x, self.y, self.z = 0, 0, 0
    	self.vx, self.vy, self.vz = 0, 0, 0
    	self.q0, self.q1, self.q2, self.q3 = 0, 0, 0, 0
    	self.wx, self.wy, self.wz = 0,0,0

    	self.roll, self.pitch, self.yaw = 0,0,0 #current roll, pitch, yaw

        self.cmd_pub = rospy.Publisher('/skid4wd/drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        self.robotpose = None
        self.robottwist =None


    def gazeboStateCb(self, msg):
        # global payload_state
        # global i, j, k
        idx = msg.name.index('skid4wd')
        self.robotpose = msg.pose[idx]
        self.robottwist = msg.twist[idx]
        # position_p = self.robot.position
        # orientation_q = self.robot.orientation
        # print(position_p)
        # print(orientation_q)

        self.x = self.robotpose.position.x
        self.y = self.robotpose.position.y
        self.z = self.robotpose.position.z

        self.q0 = self.robotpose.orientation.w
        self.q1 = self.robotpose.orientation.x
        self.q2 = self.robotpose.orientation.y
        self.q3 = self.robotpose.orientation.z

        self.vx = self.robottwist.linear.x
        self.vy = self.robottwist.linear.y
        self.vz = self.robottwist.linear.z

        self.wx = self.robottwist.angular.x
        self.wy = self.robottwist.angular.y
        self.wz = self.robottwist.angular.z

        orientation_list = [self.q1, self.q2, self.q3, self.q0]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)


    def odomCb(self, msg):
     #    self.x = msg.pose.pose.position.x
     #    self.y = msg.pose.pose.position.y
     #    self.z = msg.pose.pose.position.z

     #    self.q0 = msg.pose.pose.orientation.w
    	# self.q1 = msg.pose.pose.orientation.x
    	# self.q2 = msg.pose.pose.orientation.y
    	# self.q3 = msg.pose.pose.orientation.z

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z

        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z

        orientation_list = [self.q1, self.q2, self.q3, self.q0]
    	(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def cmd_velocity(self, vx, wz):
        # now = rospy.Time.now()  
        # self.twist.header.stamp = now
        self.twist.linear.x = vx
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0 
        self.twist.angular.z = wz

    def pub_vel(self, vx, wz):
        self.cmd_velocity(vx, wz)
        self.cmd_pub.publish(self.twist)



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
    # print("item: ", item)
    myArrayString = myArrayString + str(item) + "|"
  # print(myArrayString)
  s.send((myArrayString).encode())  
  return

def Tcp_Read_Array():
  files = s.recv(1024)
  files = files.decode()
  myArray = files.split('|')

  # for myItem in myArray:
  #   print(myItem)
  return myArray


def main():
    rospy.init_node('setpoint_node', anonymous=True)

    cnt = Controller()
    rate = rospy.Rate(20.0)

    # rospy.Subscriber('/skid4wd/drive_controller/odom', Odometry, cnt.odomCb)
    rospy.Subscriber('/gazebo/model_states', ModelStates, cnt.gazeboStateCb)

    Tcp_connect( '127.0.0.1', 17098) 

    while not rospy.is_shutdown():

        arr = [cnt.x, cnt.y, cnt.yaw, cnt.vx, cnt.wz]
        Tcp_Write_Array(arr) 

        vel = Tcp_Read_Array()
        # vel = float(vel)
        # print(vel[0])
        # print(vel[1])
        # print(type(vel[0]))

        print("----------")


        cnt.pub_vel(float(vel[0]), float(vel[1]))


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

