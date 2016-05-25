#!/usr/bin/env python
import rospy
import time # for sleep
from geometry_msgs.msg import Twist

def pub_cmd_vel(vel_pub, vx, vy, omega):
	twist = Twist()
	twist.linear.x = vx
	twist.linear.y = vy
	twist.angular.z = omega
	vel_pub.publish(twist)

if __name__ == '__main__':
	try:	
		rospy.init_node('ouijabot_tel_op')
		vel_pub = rospy.Publisher('command_velocity', Twist, queue_size=15)
		
		time.sleep(0.5)

		pub_cmd_vel(vel_pub, 0.3, 0, 0)
		time.sleep(5)

		pub_cmd_vel(vel_pub, 0, 0, 0)
		time.sleep(0.1)
		pub_cmd_vel(vel_pub, 0, 0, 0)

		# rospy.spin()
	except rospy.ROSInterruptException:
		pass
