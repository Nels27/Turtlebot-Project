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
move_cmd.angular.z = 0 #Setting the initial destination

turnAdjustment_cmd = Twist()
turnAdjustment_cmd.linear.x = 0
turnAdjustment_cmd.angular.z = 0

class GoForward():
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)
		rospy.loginfo("To stop TurtleBot CTRL+C")
		rospy.on_shutdown(self.shutdown)
		rospy.Subscriber('/odom', Odometry, self.Turning)
	#	rospy.Subscriber('/odom', Odometry, self.TurningFixed)

		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		r = rospy.Rate(5)


		for x in range(0,10):
			self.cmd_vel.publish(move_cmd)
			r.sleep()

	def Turning(self,msg): #Sets the desired angle for turning
		#print ("Please enter the angle")
		#value = input()
		global value
		print ("Please enter the angle")
		value = input() #135 degrees
		move_cmd.angular.z = value


#	def TurningFixed(self,msg): #Corrects the turning with an error adjustment in real time
#		print ("Provide turn angle")
#		zdes = cmath.rect(1,value)
#		real = msg.pose.pose.orientation.z
#		imag = msg.pose.pose.orientation.w
#
#		zcurr = real + imag * 1j;
#		zerror = zdes / zcurr
#		phase_angle = cmath.phase(zerror)

#		k - 1
#		omega = k * phase_angle;
#		turnAdjustment_cmd = Twist()
#		turnAdjustment_cmd.angular.z = radians(omega)
#		move_cmd.angular.z = value
#
	def shutdown(self):
		rospy.loginfo("Stop TurtleBot")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

if __name__ ==  '__main__':
	try:
		GoForward()
	except:
		GoForward()
