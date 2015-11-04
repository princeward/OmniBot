#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

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

# Global variables
RB_ID = 5 # rigid-body ID in the optitrack system
distance = 1000

opti = OptiTrackInterface()

dest = (4.5, 1.5)


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


def get_joystick_data():
    global distance

    pos = opti.get_pos(RB_ID)
    print("%.3f,%.3f"%(pos[0],pos[1]) )

    distance = math.sqrt((pos[0]-dest[0])*(pos[0]-dest[0]) + (pos[1]-dest[1])*(pos[1]-dest[1]))
    
    desire_ang = get_vec_ang((dest[0]-pos[0], dest[1]-pos[1]))
    
    velocity = '200'
    angle = str(int(desire_ang)).zfill(3)
    print angle
    angle_velocity = '0001'

    move_command='@a'+velocity+angle+angle_velocity+'?!'
    pub.publish(move_command)

    
    
    
    
current = 0
def current_callback(data):
    global current
    current = data.data
    rospy.loginfo("%s",data.data)

if __name__ == '__main__': 
    rospy.init_node('pc', anonymous=True)
    #rospy.Subscriber('current',String,current_callback)
    pub = rospy.Publisher('joystick',String,queue_size = 14)


    while (distance > 0.3):
        #screen.fill(WHITE)
        #textPrint.reset()
        get_joystick_data()
        #textPrint.printf(screen, "current = {}".format(current))
        #pygame.display.flip()
        #clock.tick(100)    
        sleep(0.03)

	
    move_command='@a0000000001?!'
    pub.publish(move_command)
    print "Reach destination!"
    
       
    
