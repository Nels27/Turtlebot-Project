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

roll = pitch = yaw = 0.0

class GoForward():
    def __init__(self):
        rospy.init_node('GoForward', anonymous=False)
        rospy.loginfo("To stop TurtleBot CTRL+C")
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('/odom', Odometry, self.Turning)
    #	rospy.Subscriber('/odom', Odometry, self.TurningFixed)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        r = rospy.Rate(5)
        global value
        print("Please enter the angle")
        value = input()
        move_cmd.angular.z = radians(value)
        for x in range(0,10):
            self.cmd_vel.publish(move_cmd)
            r.sleep()


        for x in range(0,10):
            self.cmd_vel.publish(turnAdjustment_cmd)
            r.sleep()

    def Turning(self,msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        desired_turn = radians(value)
        zcurr = cmath.rect(1,yaw)
        zdes = cmath.rect(1,value)
        zerror = zdes/zcurr
        zphase_error = cmath.phase(zerror)
        k=-1
        turnAdjustment_cmd.angular.z = k*zphase_error
        #print("yaw %d", yaw)
        #print("zcurr %d", zcurr)
        #print("zdes %d",zdes)
        #print("zerror %d", zerror)
        #print("zphase %d", zphase_error)

    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ ==  '__main__':
    try:
        GoForward()
    except:
        GoForward()
