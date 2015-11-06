import rospy
from std_msgs.msg import String
import pygame
import math
import time
from time import sleep

# Import optitrack functions
import sys
# add the upper directory
sys.path.append("..")
# include optitrack module
from src_optitrack.optitrack import *
from PID import PID

# Global variables
RB_ID = 5 # rigid-body ID in the optitrack system
distance = 1000
velocity = '000'
angle = '000'
opti = OptiTrackInterface()
pid_output=0
rotate_flag='0'
dest = (4.5, 1.5)
ref_yaw = 180  #set reference yaw = 0

def get_rot_dir( from_ang, to_ang):
	global rotate_flag
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
			rotate_flag='0'
		else:
			rotate_flag='1'
	else:
		if to_ang>=(from_ang-math.pi) and to_ang<from_ang:
			rotate_flag='1'
		else:
			rotate_flag='0'
		

def send_PID_command():
    global distance,velocity,angle,pid_output,yaw_degree,move_command,ref_yaw
    

    velocity ='000' #for yaw testing purposes, set velociy to 0
    angle ='000' #for yaw testing purposes, set moving angle to 0

    pid_output=str(int(p.update(yaw_degree))) #should be 3 digits, exluding "1" and "2" 

    if int(p.update(yaw_degree)) > 100:
	move_command='@a'+velocity+angle+'100'+rotate_flag+'?!'
    elif 0<=int(p.update(yaw_degree)) and int(p.update(yaw_degree)) <= 100:
	move_command='@a'+velocity+angle+pid_output.zfill(3)+rotate_flag+'?!'
    elif -100<=int(p.update(yaw_degree)) and int(p.update(yaw_degree)) < 0:
	pid_output=str(int(-p.update(yaw_degree)))
	move_command='@a'+velocity+angle+pid_output.zfill(3)+rotate_flag+'?!'
    else:
	move_command='@a'+velocity+angle+'100'+rotate_flag+'?!'
    #move_command='@a0000000001?!' 
    

if __name__ == '__main__': 
    rospy.init_node('pc', anonymous=True)
    pub = rospy.Publisher('joystick',String,queue_size = 14)    #send command data to robot
    p=PID(6,0.04,0) #initiallize PID
    p.setPoint(ref_yaw)    #intialize reference yaw point

    while(True):
	pos = opti.get_pos(RB_ID)
        yaw_degree=int(pos[5]*180/math.pi)+180
	get_rot_dir(yaw_degree*math.pi/180,ref_yaw*math.pi/180)
	print yaw_degree
	if math.fabs(yaw_degree-ref_yaw) >1:
    		send_PID_command()
	else:
		p.selferror=0.0
		move_command='@a0000000001?!' 
	#move_command='@a0000000001?!' 
	print move_command
	pub.publish(move_command)
        sleep(0.0001)
    

