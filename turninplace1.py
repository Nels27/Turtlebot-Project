#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import cmath

roll = pitch = yaw = 0.0
value = 0
move_cmd = Twist()
move_cmd.linear.x = 0
move_cmd.angular.z = 1

class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL+C")
		rospy.on_shutdown(self.shutdown)
		rospy.Subscriber('/odom', Odometry, self.ErrorHandle)

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		r = rospy.Rate(10)


		while not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			r.sleep()		
			

	def ErrorHandle(self,msg):
		print ("Provide turn angle")
		value = input()
		move_cmd.angular.z = radians(value)

	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ ==  '__main__':
	try:
		GoForward()
	except:
		GoForward()
