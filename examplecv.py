#!/usr/bin/env python

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class turtlebot_openCV():
    def __init__(self):
        self.node_name = "turtlebot_openCV"

        rospy.init_node(self.node_name)

        # Create the OpenCV display window for the RGB image
        cv.NamedWindow("Color Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Color Image", 25, 75)

        # And one for the depth image
        cv.NamedWindow("Depth Image", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Depth Image", 25, 350)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback, queue_size=1)


    def image_callback(self, data):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.convert_image(data)

        # Process the image to detect and track objects or features
        processed_image = self.process_image(frame)

        # Display the image.
        cv2.imshow(self.node_name, processed_image)

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
        cv2.imshow("Depth Image", depth_display_image)

    def convert_image(self, ros_image):

        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as e:
            print(e)

    def convert_depth_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")

            # Convert to a numpy array since this is what OpenCV uses
            depth_image = np.array(depth_image, dtype=np.float32)

            return depth_image

        except CvBridgeError as e:
            print(e)

    def process_depth_image(self, frame):
        # Just return the raw image
        return frame

    def process_image(self, frame):
        return frame

def main(args):
    try:
        turtlebot_openCV()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

