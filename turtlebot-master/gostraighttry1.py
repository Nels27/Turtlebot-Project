#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians
import cmath

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
		turn_cmd = Twist()
		turn_cmd.linear.x = 0
		turn_cmd.angular.z = 0 

		while not rospy.is_shutdown():
			self.cmd_vel.publish(move_cmd)
			self.cmd_vel.publish(turn_cmd)			
			r.sleep()
			

	def ErrorHandle(self,msg):
		zdes = cmath.rect(1,0)		
		real = msg.pose.pose.orientation.z
		imag = msg.pose.pose.orientation.w
		zcurr = real + imag*1j;
		zcurr_phase = cmath.phase(zcurr);
		zcurr_real = 1;
		zcurr = cmath.rect(zcurr_real,zcurr_phase)		

		zerror = zdes/zcurr;

		thetaerror = cmath.phase(zerror);		
		correction  = -1*thetaerror;
		print zcurr_phase

		turn_cmd = Twist()
		turn_cmd.linear.x = 0

		if correction > 10:
			turn_cmd.angular.z = radians(10)
		elif correction < -10:
			turn_cmd.angular.z = radians(-10)
		else:
			turn_cmd.angular.z = radians(correction)

	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ ==  '__main__':
	try:
		GoForward()
	except:
		GoForward()
