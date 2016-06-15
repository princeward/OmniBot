#!/usr/bin/env python
import rospy
import time
import math
import signal # for catching keyboard interrupt ctrl+c
import sys

# Standard ROS message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import Float32

# Custom ROS message
from robot_pi.msg import Wheel_Spd # custom ROS msg for wheel spd (4 floats)
from robot_pi.msg import Current # custom ROS msg for motor current (4 floats)

# Initialize ros node globally, as well as all the publishers

# Exit safely: will run when ctrl+c is pressed
def exit_handler(signal, frame):
	print 'Program ended by user!'
	sys.exit(0)

signal.signal(signal.SIGINT, exit_handler) # register the exit interrupt function


def vel2twist(vx,vy,omega):
	twist = Twist()
	twist.linear.x = vx
	twist.linear.y = vy
	twist.angular.z = omega
	return twist

class Follower:
	"""
	TBD
	"""
	
	def __init__(self):
		rospy.init_node('robot_manip_ctrl', anonymous=True)
		# variables
		self.is_auto = False
		self.freq_ctrl = rospy.Rate(20) # set the loop frequency of auto_behavior 		
		self.current = [0.0, 0.0, 0.0, 0.0] # raw current sensor data
		self.current_filter = [0.0, 0.0, 0.0, 0.0] # low-pass filter on current
		self.Ix = 0 # total current along +x axis
		self.Iy = 0 # total current along +y axis 		
		self.sum_err_Ix = 0 # used for current PID control
		self.sum_err_Iy = 0
		self.Idx_integral = 0 #####################
		self.Idy_integral = 0 #####################
		self.acc = [0.0, 0.0, 0.0] # raw 3-axis acceleration, this is very noisy
		self.acc_offset = [0.0, 0.0, 0.0] # should be applied to self.acc to remove static offset
		self.acc_filter = [0.0, 0.0, 0.0]

		# ROS publishers
		self.own_vel_pub = rospy.Publisher('command_velocity', Twist, queue_size = 15) # note the topic is relative to its own namespace
		self.temp1_pub = rospy.Publisher('temp1', Float32, queue_size = 15) # temps for Debugging 
		self.temp2_pub = rospy.Publisher('temp2', Float32, queue_size = 15) 
		self.temp3_pub = rospy.Publisher('temp3', Float32, queue_size = 15) 
		self.temp4_pub = rospy.Publisher('temp4', Float32, queue_size = 15) 
		
		# ROS subscribers
		self.auto_mode_sub = rospy.Subscriber('/is_follower_auto', Bool, self.auto_switch_callback) # this topic should be global
		self.current_sub = rospy.Subscriber('current', Current, self.current_sub_callback)
		self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_sub_callback)

	def imu_calibration(self):
		self.own_vel_pub.publish(vel2twist(0,0,0))
		print("[Info]: IMU calibrating... Do not move the robot.")
		time.sleep(3)
		self.acc_offset = [self.acc_filter[0], self.acc_filter[1], 0] # do not include offset for z axis
		print("[Info]: Done with IMU calibration")

	def current_sub_callback(self, msg):
		self.current = msg.current # read the raw current
		for i in range(4): # low-pass filter for the current
			self.current_filter[i] = 0.95*self.current_filter[i] + 0.05*self.current[i]
		
		# calculate the total current, which is based on the filter currents
		val_sqrt2 = 0.707107 # value of sqrt(2)/2
		self.Ix = val_sqrt2*(-self.current_filter[0]-self.current_filter[1]+self.current_filter[2]+self.current_filter[3])
		self.Iy = val_sqrt2*(-self.current_filter[0]+self.current_filter[1]+self.current_filter[2]-self.current_filter[3])
		
		temp = Float32()
		temp.data = self.Ix
		self.temp1_pub.publish(temp)
		temp.data = self.Iy
		self.temp2_pub.publish(temp)
	
	def imu_sub_callback(self, msg):
		# read raw Acc
		acc = msg.linear_acceleration		
		self.acc = [-(acc.x+self.acc_offset[0]), -(acc.y+self.acc_offset[1]), -acc.z] # x-axis and y-axis happens to oppose to the IMU axes

		# low-pass filter for Acc
		for i in range(3):
			self.acc_filter[i] = 0.99*self.acc_filter[i] + 0.01* self.acc[i]

		temp = Float32()
		temp.data = self.acc_filter[0]
		self.temp3_pub.publish(temp)
		temp.data = self.acc_filter[1]
		self.temp4_pub.publish(temp)		
			
	def auto_switch_callback(self, msg):
		if msg.data == True: # activate auto mode
			self.is_auto = True
			print("Auto Mode ON")
		if msg.data == False: # de-activate the auto mode
			self.own_vel_pub.publish(vel2twist(0,0,0))
			self.is_auto = False
			print("Auto Mode OFF")

	def auto_behavior(self):
		# self.own_vel_pub.publish(vel2twist(0.2,0,0.05))
		
		# current PID control
		if self.acc_filter[1] > 0.15:
			self.Idy_integral += 0.01*self.acc_filter[1]
		elif self.acc_filter[1] < -0.15:
			self.Idy_integral -= 0.01*self.acc_filter[1]
		else:
			self.Idy_integral = 0		

		Idy = 1.1 + self.Idy_integral
		if Idy > 2:
			Idy = 2
		if Idy < 0:
			Idy = 0
		err = Idy - self.Iy
		self.sum_err_Iy = self.sum_err_Iy + err
		vy = 0.7 * err + 0.05 * self.sum_err_Iy
		self.own_vel_pub.publish(vel2twist(0,vy,0))
		
		self.freq_ctrl.sleep() # maintain a specified loop frequency
	

if __name__ == '__main__':
	print "hello world, this is the follower robot created by manip_follower.py"
	
	follower = Follower()
	follower.imu_calibration()

	while True: # infinite loop
		while follower.is_auto:
			follower.auto_behavior()
	
	rospy.spin()

	    
