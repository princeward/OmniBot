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

#############   Optitrack Interface ################
opti = OptiTrackInterface()
RB_ID = 5 # rigid-body ID in the optitrack system

#############  Destination ###########
dest = (1.5, 1.5)
ref_yaw_err = 0.0  #set reference yaw = 0

############# Global variables   ##############
angle ='000'
ref_distance_err = 0.0
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

####### Position PID Calculation##########

def Calc_PID_Pos():

    global angle

    distance = math.sqrt((pos[0]-dest[0])*(pos[0]-dest[0]) + (pos[1]-dest[1])*(pos[1]-dest[1]))	#distance between center of body to the destination

    desire_ang = get_vec_ang((dest[0]-pos[0], dest[1]-pos[1]))	#angle between body frame and world frame

    angle = str(int(desire_ang)).zfill(3) #convert desire_ang to string with zero fill in
    
    if hPID_position.update(distance)<300: # maximum speed sets to 300

		angle_pid_vel_Op=str(int(hPID_position.update(distance)))

    else:

    	angle_pid_vel_Op='300'

    return angle_pid_vel_Op #velocity ouput through Postion PID

###### Yaw_angle PID Calculation#########
 def Calc_PID_Ang():




 	return pid_ang_out, direction
####### This function combines all the PID and send out the command########
 def SendCommand():
 	vel = Calc_PID_Pos()
 	ang, direction = Calc_PID_Ang()
 	move_command = XXXXXX
 	pub.publish(move_command)

    
####### Main Function with Initialization and communication tools#############
if __name__ == '__main__': 

    rospy.init_node('pc', anonymous=True)

    pub = rospy.Publisher('joystick',String,queue_size = 14)    #send command data to robot

    hPID_position = PID(1,0,0)     # Input position PID value

    hPID_position.setPoint(ref_distance_err)    #intialize reference distance error reference

    hPID_angle = PID(1,0,0)		# Input yaw_angle PID value

	hPID_angle.setPoint(ref_yaw_err)	#Input yaw_angle PID value
    
    while(True):
		pos = opti.get_pos(RB_ID)	# reading all the information from optitrack (x,y,z,...)
		print distance
		if distance >0.01:
			send_PID_command()
		else:
			p.selferror=0.0
			move_command='@a0000000001?!' 
		print move_command
		pub.publish(move_command)
		sleep(0.0001)
    

