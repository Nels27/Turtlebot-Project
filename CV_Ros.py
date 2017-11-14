#!/usr/bin/env python

from __future__ import print_function
import roslib
roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):

    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("input_rgb_image",Image,self.image_callback)
    self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback)

  def image_callback(self,data):
    # Use cv_bridge() to convert the ROS image to OpenCV format
    # Convert the ROS image to OpenCV format using a cv_bridge helper function
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError as e:
            print(e)

        # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
        depth_array = np.array(depth_image, dtype=np.float32)

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

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)