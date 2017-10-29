#!/usr/bin/env python
# Imported necessary information
import rospy
from cmath import rect
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians
import cmath

turninitial_cmd = Twist()
turninitial_cmd.linear.x = 0
turninitial_cmd.angular.z = 0

turnfix_cmd = Twist()
turnfix_cmd.linear.x = 0
turnfix_cmd.angular.z = 0

class GoForward():
    def __init__(self):
        rospy.init_node('GoForward', anonymous=False)
        rospy.loginfo("To stop TurtleBot CTRL+C")
        rospy.on_shutdown(self.shutdown)
        #subscribe to the Odometer wutg a Turning and Turning Fix class
        rospy.Subscriber('/odom', Odometry, self.Turning)
        rospy.Subscriber('/odom', Odometry, self.Turning_fix)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        r = rospy.Rate(10);

        while not rospy.is_shutdown():
            self.cmd_vel.publish(turninitial_cmd)
            self.cmd_vel.publish(turnfix_cmd)
            r.sleep(5)

    def Turning(self,msg):
        global value
        print ("Please enter the angle")
        value = input()
        turninitial_cmd.angular.z = radians(value)
        rospy.loginfo("This is the value:")
        print value

    def Turning_fix(self,msg):
        zdes = cmath.rect(1, value)
        real = msg.pose.pose.orientation.z
        imag = msg.pose.pose.orientation.w
        zcurr = real + imag * 1j;
        zerror = zdes / zcurr
        phase_angle = cmath.phase(zerror);

        k = 1
        omega = k * phase_angle;
        turnfix_cmd = Twist()
        turnfix_cmd.angular.z = radians(omega)

    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")
