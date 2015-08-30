#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
from time import sleep
from WrapMsg import *
 
def joystick_callback(data):
	vel_mag = float(data.data[2:5])
	vel_dir = float(data.data[5:8])
	omega = float(data.data[8:11])
	if data.data[11] == '2':
		omega=-omega
	#print vel_mag, vel_dir, omega

	ser.write(Wrap_Msg_A(vel_mag, vel_dir, omega))

def publish_current():
        pub.publish(current)
        rospy.loginfo(current)


if __name__ == '__main__':
	ser = serial.Serial('/dev/ttyAMA0',115200)	
	rospy.init_node('robot', anonymous=True)
        pub = rospy.Publisher('current', String, queue_size=15)
	rospy.Subscriber("joystick", String, joystick_callback)

 
	while True:
		check=ser.read(2)
	        if check[0]=='@' and check[1]=='1':
			current = ser.read(13)
			if current[12]=='!':
				publish_current()
		sleep(0.001)
    
