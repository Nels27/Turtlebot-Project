#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class Follower:

        def __init__(self):

                self.image_pub = rospy.Publisher("image_topic_2", Image)

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
                self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)
                self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)

                self.twist = Twist()

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_orange = numpy.array([0, 100, 100])
                upper_orange = numpy.array([20, 255, 255])
                mask = cv2.inRange(hsv, lower_orange, upper_orange)

                h, w, d = image.shape

                M = cv2.moments(mask)
                if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        print("X - value",cx)
                        print("Y - Value",cy)
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
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
                #cv2.imshow("window", image)
                cv2.waitKey(3)

        def depth_callback(self, ros_image):
            # Use cv_bridge() to convert the ROS image to OpenCV format
            try:
                # Convert the depth image using the default passthrough encoding
                depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
            except CvBridgeError as e:
                print(e)

            # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
            depth_array = numpy.array(depth_image, dtype=numpy.float32)

            # Normalize the depth image to fall between 0 (black) and 1 (white)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

            # Process the depth image
            depth_display_image = self.process_depth_image(depth_array)

            # Display the result
            cv2.imshow("Depth Image Scanner", depth_display_image)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(depth_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

        def process_depth_image(self, frame):
            # Just return the raw image
            return frame

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()