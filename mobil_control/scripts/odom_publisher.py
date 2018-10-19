#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from math import sin,cos, pi,sqrt,pow
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String




class Localization(object):
	def __init__(self):
		rospy.init_node('odom_publisher')


		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.th = 0.0
		self.left=0.0
		self.right=0.0
		self.left_old=0.0
		self.right_old=0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vth = 0.0
		self.left_wheel=0.0
		self.right_wheel=0.0
		self.dist_btw_wheels=0.018
		self.current_time =  rospy.Time.now()
		self.last_time =  rospy.Time.now()
		self.odom_cur=Odometry()
		self.odom_pub = rospy.Publisher('/odometry/wheel', Odometry, queue_size = 10)
		rospy.Subscriber('/mobil_serial_encoder', String, self.callback)
		self.controller()
	   
	 
 
	def callback(self,data):
		self.splitted=data.data.split(',')  

		if(self.splitted[0]=='S'):
			
			self.left=float(self.splitted[1])/1000
			self.right=float(self.splitted[2])/1000
			
			#print(str(self.left)+","+str(self.right))
	  
	def controller(self):
		self.rate = rospy.Rate(20) #10 Hz
		while not rospy.is_shutdown():
			self.current_time = rospy.Time.now()
			self.dt = (self.current_time - self.last_time).to_sec()

			self.left_wheel = (self.left-self.left_old)/self.dt
			self.right_wheel = (self.right-self.right_old)/self.dt
			#print(str(self.left_wheel)+","+str(self.right_wheel))
			self.vx =  ((self.right_wheel+self.left_wheel)/2) 
			self.vth  = ((self.right_wheel-self.left_wheel)/self.dist_btw_wheels)


			self.delta_x = (self.vx * cos(self.th) + self.vy * sin(self.th)) * self.dt
			self.delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
			self.delta_th = self.vth * self.dt
			self.x += self.delta_x
			self.y += self.delta_y
			self.th +=self.delta_th    

			print('distance: '+str(sqrt(pow(self.x,2)+pow(self.y,2)))+'yaw: '+str(self.th)+'vx:'+str(self.vx)+'vth:'+str(self.vth))
			self.q = tf.transformations.quaternion_from_euler(0, 0, self.th)
			# next, we'll publish the odometry message over ROS
			self.odom = Odometry()
			self.odom.header.stamp = self.current_time
			self.odom.header.frame_id = "odom"
			# set the position
			self.odom.pose.pose = Pose(Point(self.x , self.y, self.z), Quaternion(*self.q))
			self.odom.child_frame_id = "base_link"          
			self.last_time = self.current_time
			self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
			# Publisher(s) 
			#print(self.odom)
			self.left_old=self.left
			self.right_old=self.right
			self.odom_pub.publish(self.odom) 
			self.rate.sleep()     

if __name__ == '__main__':
	Localization()
