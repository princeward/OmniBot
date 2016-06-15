#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

rospy.init_node('ouijabot_tel_op')
bot1_vel_pub = rospy.Publisher('bot1_vel', Twist, queue_size=15)
bot2_vel_pub = rospy.Publisher('bot2_vel', Twist, queue_size=15)
bot3_vel_pub = rospy.Publisher('bot3_vel', Twist, queue_size=15)
bot4_vel_pub = rospy.Publisher('bot4_vel', Twist, queue_size=15)
follower_auto_state_pub = rospy.Publisher('is_follower_auto', Bool, queue_size=15)

is_follower_auto = Bool()

def vel2twist(vx,vy,omega):
	twist = Twist()
	twist.linear.x = vx
	twist.linear.y = vy
	twist.angular.z = omega
	return twist

def joystick_handle(msg):
	# global bot2_vel_pub
	if 	msg.buttons[0] == 1: # robot 2 goes to autonomous mode
		#bot2_vel_pub.publish(vel2twist(0.2,0,0.05))	
		# mark the followers as autonomous mode
		is_follower_auto.data = True
		follower_auto_state_pub.publish(is_follower_auto)
	if msg.buttons[1] == 1: # robot 2 stops
		#bot2_vel_pub.publish(vel2twist(0,0,0))	
		#bot2_vel_pub.publish(vel2twist(0,0,0))	
		# mark the end of autonomous mode of the follower robots
		is_follower_auto.data = False
		follower_auto_state_pub.publish(is_follower_auto)
	if msg.buttons[7] == 1: # manual control for only followers
		twist_vel = vel2twist(-msg.axes[0],msg.axes[1],msg.axes[2])
		bot2_vel_pub.publish(twist_vel)
		bot1_vel_pub.publish(twist_vel)
		bot4_vel_pub.publish(twist_vel)
	if msg.buttons[9] == 1: # manual control for only leader
		bot3_vel_pub.publish(vel2twist(-msg.axes[0],msg.axes[1],msg.axes[2]))
	if msg.buttons[6] == 1:	# manual control for all robots
		twist_vel = vel2twist(-msg.axes[0],msg.axes[1],msg.axes[2])
		bot1_vel_pub.publish(twist_vel)
		bot2_vel_pub.publish(twist_vel)
		bot3_vel_pub.publish(twist_vel)
		bot4_vel_pub.publish(twist_vel)


if __name__ == '__main__':
	try:	
		joy_sub = rospy.Subscriber('in', Joy, joystick_handle)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass



