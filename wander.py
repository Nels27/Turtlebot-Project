#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from kobuki_msgs.msg import CliffEvent


class Scan_msg:
    def __init__(self):
        '''Initializes an object of this class.

        The constructor creates a publisher, a twist message.
        3 integer variables are created to keep track of where obstacles exist.
        3 dictionaries are to keep track of the movement and log messages.'''
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.BumperEventCallback)
        rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.WheelEventCallback)
        rospy.Subscriber("/mobile_base/events/cliff",CliffEvent, self.CliffEventCallback)
        self.msg = Twist()
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.sect_4 = 0
        self.sect_5 = 0
        self.bhit = 0
        self.wheelhit = 0
        self.cliffhit = 0
        self.ang = {0: 0, 1: -0.5, 10: -1.2, 100: 1.5, 1000: 1.2, 10000: 0.5, 11: -1.2, 110: -1.2, 1100: 1.2, 11000: 1.5, 10001: 0, 111: -1.2, 1110: 1.5, 11100: 1.5, 11001: 1.2, 10011: 0.5, 1111: -1.5, 11110: 1.5, 11101: 0.5, 11011: 0, 10111: -0.5, 11111: 1.7}
        self.fwd = {0: 0.25, 1: 0, 10: 0, 100: 0, 1000: 0, 10000: 0, 11: 0, 110: 0, 1100: 0, 11000: 0, 10001: 0.25, 111: 0, 1110: 0, 11100: 0, 11001: 0, 10011: 0, 11110: 0, 11101: 0, 11011: 0.25, 10111: 0,1111: 0, 11111: 0}
        self.dbgmsg = {0: 'Move Forward', 1: 'Veer Right', 10: 'Veer Right', 100: 'Veer Left', 1000: 'Veer Left', 10000: 'Veer Left', 11: 'Veer Right', 110: 'Veer Right', 1100: 'Veer Left', 11000: 'Veer Left', 10001: 'Move Forward', 111: 'Veer Right', 1110: 'Veer Left', 11100: 'Veer Left', 11001: 'Veer Left', 10011: 'Veer Left', 11110: 'Veer Left', 11101: 'Veer Left', 11011: 'Move Forward', 10111: 'Veer Right',1111: 'Veer Right', 11111: 'Veer Right'}

    def reset_sect(self):
        '''Resets the below variables before each new scan message is read'''
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.sect_4 = 0
        self.sect_5 = 0

    def BumperEventCallback(self,data):
        if ( data.state == BumperEvent.PRESSED ) :
            self.bhit = 1
        elif (data.state == BumperEvent.RELEASED):
            self.bhit =0
        else:
            self.bhit = 0

    def WheelEventCallback(self,data):
        if (data.state == WheelDropEvent.RAISED):
            self.wheelhit = 0
        elif (data.state == WheelDropEvent.DROPPED):
            self.wheelhit = 1
        else:
            self.wheelhit = 0

    def CliffEventCallback (self, data):
        if (data.state == CliffEvent.CLIFF):
            self.cliffhit = 1
        elif (data.state == CliffEvent.FLOOR):
            self.cliffhit = 0
        else:
            self.cliffhit = 0




    def sort(self, laserscan):
        '''Goes through 'ranges' array in laserscan message and determines
        where obstacles are located. The class variables sect_1, sect_2,
        and sect_3 are updated as either '0' (no obstacles within 0.7 m)
        or '1' (obstacles within 0.7 m)

        Parameter laserscan is a laserscan message.'''
        entries = len(laserscan.ranges)
        for entry in range(0, entries):
            if 0.4 < laserscan.ranges[entry] < 0.75:
                self.sect_1 = 1 if (0 < entry < entries / 5) else 0
                self.sect_2 = 1 if (entries / 5 < entry < 2* entries / 5) else 0
                self.sect_3 = 1 if (2*entries / 5 < entry < 3*entries/ 5) else 0
                self.sect_4 = 1 if (3*entries / 5 < entry < 4*entries / 5) else 0
                self.sect_5 = 1 if (4*entries / 5 < entry < entries) else 0
        rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(
            self.sect_3) + " sect_4: " + str(self.sect_4) + " sect_5: " + str(self.sect_5))

    def movement(self, sect1, sect2, sect3):
        '''Uses the information known about the obstacles to move robot.

        Parameters are class variables and are used to assign a value to
        variable sect and then	set the appropriate angular and linear
        velocities, and log messages.
        These are published and the sect variables are reset.'''
        sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3) + str(self.sect_4) + str(self.sect_5))
        rospy.loginfo("Sect = " + str(sect))

        self.msg.angular.z = self.ang[sect]
        self.msg.linear.x = self.fwd[sect]
        rospy.loginfo(self.dbgmsg[sect])
        if (self.bhit == 1 or self.wheelhit == 1 or self.cliffhit == 1):
            self.msg.angular.z = 0
            self.msg.linear.x = 0
            self.pub.publish(self.msg)
        else:
            self.pub.publish(self.msg)

        self.reset_sect()

    def for_callback(self, laserscan):
        '''Passes laserscan onto function sort which gives the sect
        variables the proper values.  Then the movement function is run
        with the class sect variables as parameters.

        Parameter laserscan is received from callback function.'''
        self.sort(laserscan)
        self.movement(self.sect_1, self.sect_2, self.sect_3)


def call_back(scanmsg):
    '''Passes laser scan message to for_callback function of sub_obj.

    Parameter scanmsg is laserscan message.'''
    sub_obj.for_callback(scanmsg)


def listener():
    '''Initializes node, creates subscriber, and states callback
    function.'''
    rospy.init_node('navigation_sensors')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber('/scan', LaserScan, call_back)
    rospy.spin()


if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run'''
    sub_obj = Scan_msg()
    listener()
