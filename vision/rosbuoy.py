#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ColorHighlighterNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('color_highlighter', anonymous=True)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to DepthAI camera topic
        self.image_sub = rospy.Subscriber("/oak/rgb/image_raw", Image, self.image_callback)

        # Publisher for processed images
        self.image_pub = rospy.Publisher("/processed/image_raw", Image, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Process the frame (highlight colors)
        processed_image = self.process_frame(cv_image)

        try:
            # Publish processed frame
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_frame(self, frame): 
        # Convert image to HSV and apply a Gaussian blur
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_hsv = cv2.GaussianBlur(frame_hsv, (11, 11), 5)

        # Define red color range in HSV and create masks
        RED_LOWER1 = np.array([0, 100, 20])
        RED_UPPER1 = np.array([10, 255, 255])
        RED_LOWER2 = np.array([160, 100, 20])
        RED_UPPER2 = np.array([180, 255, 255])

        lower_mask = cv2.inRange(frame_hsv, RED_LOWER1, RED_UPPER1)
        upper_mask = cv2.inRange(frame_hsv, RED_LOWER2, RED_UPPER2)
        mask = lower_mask + upper_mask

        # Compute the final image showing only the red areas
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour and draw it
            max_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 3)

            # Optionally draw an enclosing circle around the largest contour
            if debug_drawEnclosingCircle:
                (x, y), radius = cv2.minEnclosingCircle(max_contour)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(frame, center, radius, (255, 0, 0), 2)

        # Return the frame with the largest contour highlighted
        return frame


if __name__ == '__main__':
    try:
        node = ColorHighlighterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
