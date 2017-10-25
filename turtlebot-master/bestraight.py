#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import cmath

roll = pitch = yaw = 0.0
turn_cmd = Twist()
turn_cmd.linear.x = 0
turn_cmd.angular.z = 0 

class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL+C")
		rospy.on_shutdown(self.shutdown)
		rospy.Subscriber('/odom', Odometry, self.ErrorHandle)

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		r = rospy.Rate(10);
		move_cmd = Twist()
		move_cmd.linear.x = 0.2
		move_cmd.angular.z = 0

		while not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			self.cmd_vel.publish(turn_cmd)
					
			

	def ErrorHandle(self,msg):
		global roll, pitch, yaw
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] 
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		zdes = 0.0784;
		zerror = zdes - yaw

		if zerror > 0.5:
			turn_cmd.angular.z = 0.1
		elif zerror < -0.5:
			turn_cmd.angular.z = -0.1
		else:
			turn_cmd.angular.z = zerror

	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ ==  '__main__':
	try:
		GoForward()
	except:
		GoForward()
