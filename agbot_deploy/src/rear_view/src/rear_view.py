#!/usr/bin/env python
# USAGE
# python real_time_object_detection.py --prototxt MobileNetSSD_deploy.prototxt.txt --model MobileNetSSD_deploy.caffemodel

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2
import roslib
import sys
import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import os
# maxPixel = rospy.get_param("/maxPixel")
maxPixel = 400
vs = VideoStream(src=0).start()

def CVControl():




	# initialize the list of class labels MobileNet SSD was trained to
	# detect, then generate a set of bounding box colors for each class


	rospy.init_node("rear_view", anonymous = True)
	image_pub = rospy.Publisher("rear_cv",Image, queue_size = 10)

	rate = rospy.Rate(20)
	bridge = CvBridge()

	# loop over the frames from the video stream
	while not rospy.is_shutdown():
		# grab the frame from the threaded video stream and resize it
		# to have a maximum width of 400 pixels
		frame = vs.read()
		frame = imutils.resize(frame, width=maxPixel)

		# grab the frame dimensions and convert it to a blob

		image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))		# update the FPS counter
		#fps.update()
        ros.sleep(rate)


if __name__ == '__main__':
	CVControl()
# stop the timer and display FPS information
# fps.stop()
# print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
# print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
