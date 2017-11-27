#!/usr/bin/env python


#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image
#to be able to see only the yellow line and then follow that line
#It uses an approach called proportional and simply means

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                        Twist, queue_size=1)

                self.twist = Twist()

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                #this actually for orange
                lower_orange = numpy.array([0, 100, 100])
                upper_orange = numpy.array([20, 255, 255])
                mask = cv2.inRange(hsv, lower_orange, upper_orange)

                h, w, d = image.shape

                M = cv2.moments(mask)
                if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
#The proportional controller is implemented in the following four lines which
#is reposible of linear scaling of an error to drive the control output.
                        err = cx - w/2
                        foward_err = cy - h/2
                        #print("m00",M['m00'])
                        #print("m10",M['m10'])
                        #print("m01",M['m01'])
                        limitaion = int(M['m00'])
                        print (limitaion)
                        if (limitaion < 99999):
                                self.twist.angular.z = 0
                                self.twist.angular.x = 0
                        else:
                                self.twist.angular.z = -float(err)/100
                                self.twist.linear.x = -float(foward_err)/2000

                        self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.waitKey(3)

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()