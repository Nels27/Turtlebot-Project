#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('camera/depth/image_raw',Image, self.image_callback)


                self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)

                self.twist = Twist()

        def image_callback(self, msg, ros_image):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_thresh = numpy.array([0, 100, 100])
                upper_thresh = numpy.array([20, 255, 255])
                mask = cv2.inRange(hsv, lower_thresh, upper_thresh)

                h, w, d = depth_image.shape

                depth_array = numpy.array(depth_image, dtype=numpy.float32)
                cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

                M = cv2.moments(mask)
                if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(depth_array, (cx, cy), 20, (0,0,255), -1)
                        err = cx - w/2
                        foward_err = cy - h/2
                        limitaion = int(M['m00'])
                        if (limitaion < 999999):
                                self.twist.angular.z = 0
                                self.twist.linear.x = 0
                        else:
                                self.twist.angular.z = -float(err)/100
                                self.twist.linear.x = -float(foward_err)/500

                        self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.waitKey(3)

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
