import rospy
from std_msgs.msg import String
import pygame
import math
import time
from time import sleep

import signal # for catching keyboard interrupt ctrl+c


# Import optitrack functions
import sys
# add the upper directory
sys.path.append("..")
# include optitrack module
from src_optitrack.optitrack import *
from PID import PID

##### ROS Initialization #####
rospy.init_node('pc', anonymous=True)
pub = rospy.Publisher('joystick',String,queue_size = 14)    #send command data to robot

#############   Optitrack Interface ################
opti = OptiTrackInterface()
RB_ID = 1 # rigid-body ID in the optitrack system

#############  Destination ###########
dest = (1.5, 1.5)


############# Global variables   ##############
angle ='000'

########Function gets the shortest world frame angle or direction#########
def get_vec_ang(vec):
	if (0 == vec[0]): # vx = 0
		if (vec[1] > 0):
			return 90
		elif (vec[1] < 0):
			return 270
		else:
			return 0
	else: # vx != 0
		if(0 == vec[1]): # vy = 0
			if(vec[0] > 0):
				return 0
			else:
				return 180
		else: # vx != 0, vy != 0
			temp = math.fabs(vec[1]/vec[0])
			if ( vec[0]>0 and vec[1]>0 ): # 1
				return math.atan(temp) * 180 / math.pi
			elif ( vec[0]<0 and vec[1]>0 ): # 2
				return (math.pi-math.atan(temp)) * 180 / math.pi
			elif ( vec[0]<0 and vec[1]<0 ): # 3
				return (math.pi+math.atan(temp)) * 180 / math.pi
			else:
				return (2*math.pi-math.atan(temp)) * 180 / math.pi

def get_rot_dir_str( from_ang, to_ang):
	from_ang = from_ang*math.pi/180
	to_ang = to_ang*math.pi/180

	if from_ang <0:
		from_ang = from_ang +2*math.pi
	if from_ang> 2*math.pi:
		from_ang = from_ang-2*math.pi
	if to_ang <0:
		to_ang = to_ang +2*math.pi
	if to_ang>2*math.pi:
		to_ang = to_ang -2*math.pi

	if from_ang <=math.pi:
		if to_ang>from_ang and to_ang<=(from_ang+math.pi):
			return '2'
		else:
			return '1'
	else:
		if to_ang>=(from_ang-math.pi) and to_ang<from_ang:
			return '1'
		else:
			return '2'


# Exit safely: will run when ctrl+c is pressed
def exit_handler(signal, frame):
	move_command = "@a0000000001?!"
	pub.publish(move_command)
	time.sleep(0.5)
	pub.publish(move_command)
	print move_command
	print 'Program ended!'
	sys.exit(0)

signal.signal(signal.SIGINT, exit_handler) # register the exit interrupt function

####### Position PID Calculation##########

def Calc_PID_Pos():

    global angle


    
    if hPID_position.update(distance)<300: # maximum speed sets to 300

		angle_pid_vel_Op=str(int(hPID_position.update(distance)))

    else:

    	angle_pid_vel_Op='300'

    return angle_pid_vel_Op #velocity ouput through Postion PID


# ####### This function combines all the PID and send out the command########
#  def SendCommand():
#  	vel = Calc_PID_Pos()
#  	ang, direction = Calc_PID_Ang()
#  	move_command = XXXXXX
#  	pub.publish(move_command)

    
###### Main Function with Initialization and communication tools#############
if __name__ == '__main__': 
	
	hPID_position = PID(5,0.5,0)     # Input position PID value
	hPID_position.setPoint(0)    #intialize reference distance error reference

	#hPID_angle = PID(1,0,0)		# Input yaw_angle PID value
	#hPID_angle.setPoint(ref_yaw_err)	#Input yaw_angle PID value


    
	while(True):
		pos = opti.get_pos(RB_ID)	# reading all the information from optitrack (x,y,z,...)

		##############################################
		distance = math.sqrt((pos[0]-dest[0])*(pos[0]-dest[0]) + (pos[1]-dest[1])*(pos[1]-dest[1]))	#distance between center of body to the destination
		desire_ang = get_vec_ang((dest[0]-pos[0], dest[1]-pos[1]))	# in degree

		vel_mag_pid = 50*math.fabs(hPID_position.update(distance))
		if vel_mag_pid > 200: # limit the speed
			vel_mag_pid = 200
		
		vel_str_cmd = str(int(vel_mag_pid)).zfill(3)
		vel_ang_str_cmd = str(int(desire_ang)).zfill(3)


		##############################################

		desire_robot_heading = 45
		yaw = pos[5]*180/math.pi # in degree, [-180,180]
		
		print get_rot_dir_str(yaw,desire_robot_heading)

		## 1. generate angular velocity command by PID
		## 2. combine all commends to a string command, and publish
		## 3. Wrap up everything to function




		##############################################

		move_command='@a'+vel_str_cmd+vel_ang_str_cmd+'000'+'1'+'?!'

		#pub.publish(move_command)

		
		sleep(0.001)
