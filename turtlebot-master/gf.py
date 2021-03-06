#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

class GoForward():
    def __init__(self,safety):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)
        rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.BumperEventCallback)

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.safety = 0

	#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        f = rospy.Rate(0.5);
        # Twist is a datatype for velocity (All the motions of the robot)
        move_cmd = Twist()
	# let's go forward at 0.1 m/s
        move_cmd.linear.x = 0.1
	# let's turn at 0 radians/s
	move_cmd.angular.z = 0

    move_stop = Twist()
    
   
    #safety 
    # GoFwd = 0
    # Wait = 1
    # Stop = 2
    #safety = ["GoFwd","Wait","Stop"]
    #bhit
    # None = 0
    # left = 1
    # Center = 2
    # right = 3
    #bhit = ["none","left","right","center"]
	# as long as you haven't ctrl + c keeping doing...
    while not rospy.is_shutdown():
        
	    # publish the velocity
        if self.safety == 0:
            self.cmd_vel.publish(move_cmd)
            if (bhit>1):
                self.safety = 1
        elif self.safety == 1:
            self.cmd_vel.publish(move_stop)
            if (bhit==0):
                self.safety = 2
            else:
                self.safety = 1
        elif self.safety == 2:
            f.sleep() #Stops it for 2 seconds
            self.safety  = 1
	    # wait for 0.1 seconds (10 HZ) and publish again

    def BumperEventCallback(self,data):
        global bhit
        if ( data.state == BumperEvent.RELEASED ) :
            state = "released"
            bhit = 0
        else:
            state = "pressed"  
        if ( data.bumper == BumperEvent.LEFT ) :
            bumper = "left bumper"
            bhit = 1
        elif ( data.bumper == BumperEvent.CENTER ) :
            bumper = "center bumper"
            bhit = 2
        else:
            bumper = "right bumper"
            bhit = 3
        rospy.loginfo("The %s was %s."%(bumper, state))

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")
