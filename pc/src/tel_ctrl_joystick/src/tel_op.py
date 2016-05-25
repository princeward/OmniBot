#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def callback(msg, vel_pub):
	twist = Twist()
	twist.linear.x = -msg.axes[0]
	twist.linear.y = msg.axes[1]
	twist.angular.z = msg.axes[2]
	vel_pub.publish(twist)

if __name__ == '__main__':
	try:	
		rospy.init_node('ouijabot_tel_op')
		vel_pub = rospy.Publisher('out', Twist, queue_size=15)
		joy_sub = rospy.Subscriber('in', Joy, callback, vel_pub) # vel_pub is the callback argument
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
