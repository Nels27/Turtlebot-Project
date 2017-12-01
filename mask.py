#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2

# Bounds for a red object
upperBound = (123, 123, 252)
lowerBound = (1, 0, 176)

def find_red(image_message):
    global keep_going
    bridge = cv_bridge.CvBridge()
    image = None
    try:
        image = bridge.imgmsg_to_cv2(image_message, "bgr8")  # convert image message to OpenCV image matrix
    except cv_bridge.CvBridgeError as e:
        rospy.logerr(e.message)
        print (e.message)

    if image is not None:
        mask = cv2.inRange(image, lowerBound, upperBound)  # create a mask layer based on color bounds
        mask = cv2.erode(mask, None, iterations=2)  # make the selected object in the form a four sided object
        mask = cv2.dilate(mask, None, iterations=2)

        (contours,_) =  cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours is not None and len(contours) > 0:  # if a contour is found...
            c = max(contours, key=cv2.contourArea)  # let c be the largest contour

            # approximate the centroid of the contour
            moments = cv2.moments(c)
            centroid_x = int(moments["m10"] / moments["m00"])  # x coord of centroid of object
            centroid_y = int(moments["m01"] / moments["m00"])  # y coord of centroid of object
            object_centroid = (centroid_x, centroid_y)

            # draw the contour and centroid of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (centroid_x, centroid_y), 7, (255, 255, 255), -1)
            cv2.putText(image, "center", (centroid_x - 20, centroid_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow("Red Object Found", image)  # show the image
        cv2.waitKey(1)  # refresh contents of image frame

if __name__ == "__main__":
    rospy.init_node("find_object")  # initialize the node
    rospy.Subscriber("camera/rgb/image_raw", Image, find_red)  # camera subscriber
    rospy.spin()  # keeps the script from exiting until the node is killed
