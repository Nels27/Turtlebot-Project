#!/usr/bin/env python
# Imported necessary information
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians
import cmath

turn_cmd = Twist()
turn_cmd.linear.x = 0
turn_cmd.angular.z = 0

class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL+C")
		rospy.on_shutdown(self.shutdown)
		#subscribe to the Odometer
		rospy.Subscriber('/odom', Odometry, self.Turning)

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		r = rospy.Rate(10);
		move_cmd = Twist()
		move_cmd.linear.x = 0
		move_cmd.angular.z = 0.2

		while not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			self.cmd_vel.publish(turn_cmd)

	def Turning(self,msg):
        print "Please enter the angle"
		zdes = cmath.rect(1,0)
		real = msg.pose.pose.orientation.z
		imag = msg.pose.pose.orientation.w
		zcurr = real + imag*1j;

		phase_angle = cmath.phase(zerror);

		turn_cmd = Twist()
		turn_cmd.linear.x = 0

	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ ==  '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")
