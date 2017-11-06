#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Scan_msg:
    def __init__(self):
        rospy.init_node('Scan_msg', anonymous=False)
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.for_callback)
        self.msg = Twist()
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.ang = {000: 0, 001: -1.2, 10: -1.2, 11: -1.2, 100: 1.5, 101: 1.0, 110: 1.0, 111: 1.2}
        self.fwd = {000: .25, 1: 0, 10: 0, 11: 0, 100: 0.1, 101: 0, 110: 0, 111: 0}
        self.dbgmsg = {0: 'Move forward', 1: 'Veer right', 10: 'Veer right', 11: 'Veer right', 100: 'Veer left',
                       101: 'Veer left', 110: 'Veer left', 111: 'Veer right'}

        while not rospy.is_shutdown():
            self.pub.publish(self.msg)

    def reset_sect(self):
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0

    def sort(self, laserscan):
        entries = len(laserscan.ranges)
        print(entries)
        for entry in range(0, entries):
            if 0.4 < laserscan.ranges[entry] < 0.75:
                if (0 < entry < entries / 3):
                    self.sect_1 = 1
                else:
                    self.sect_1 = 0
                if (entries / 3 < entry < entries / 2):
                    self.sect_2 = 1
                else:
                    self.sect_2 = 0
                if (entries / 2 < entry < entries):
                    self.sect_3 = 1
                else:
                    self.sect_3 = 0
            else:
                self.sect_1 = 0
                self.sect_2 = 0
                self.sect_3 = 0

        rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(
            self.sect_3))

    def movement(self, sect1, sect2, sect3):
        sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
        rospy.loginfo("Sect = " + str(sect))

        self.msg.angular.z = self.ang[sect]
        self.msg.linear.x = self.fwd[sect]
        rospy.loginfo(self.dbgmsg[sect])
        self.pub.publish(self.msg)
        print (self.msg.angular.z)
        print (self.msg.linear.x)

        self.reset_sect()

    def for_callback(self, laserscan):
        self.sort(laserscan)
        self.movement(self.sect_1, self.sect_2, self.sect_3)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.pub.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


# def call_back(scanmsg):
#     sub_obj.for_callback(scanmsg)
#
#
# def listener():
#     rospy.init_node('navigation_sensors')
#     rospy.loginfo("Subscriber Starting")
#     sub = rospy.Subscriber('/scan', LaserScan, call_back)
#     rospy.spin()


if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run'''
    sub_obj = Scan_msg()
    #listener()
