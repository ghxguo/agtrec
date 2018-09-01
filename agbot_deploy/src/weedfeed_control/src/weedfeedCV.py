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
maxPixel = 400
vs1 = VideoStream(src=0).start()
vs2 = VideoStream(src=1).start()
#vs3 = VideoStream(src=2).start()

def CVControl():
	rspkg = rospkg.RosPack()
 	prototxt_path = os.path.join(rspkg.get_path('melonCV'),"src","MNSSD_melon_1200000.prototxt")
 	caffemodel_path = os.path.join(rspkg.get_path('melonCV'),"src","MNSSD_melon_1200000.caffemodel")
	init_confidence = 0.2
	# initialize the list of class labels MobileNet SSD was trained to
	# detect, then generate a set of bounding box colors for each class
	CLASSES = ["background", "corn", "cocklebur", "pigweed", "ragweed"]
	COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

	# load our serialized model from disk
	print("[INFO] loading model...")
	net = cv2.dnn.readNetFromCaffe(prototxt_path, caffemodel_path)

	# initialize the video stream, allow the cammera sensor to warmup,
	# and initialize the FPS counter
	print("[INFO] starting video stream...")

	time.sleep(2.0)
	# fps = FPS().start()




	rospy.init_node("real_time_object_detection", anonymous = True)
	image_pub_left = rospy.Publisher("wf_box_left",Image, queue_size = 10)
	image_pub_mid = rospy.Publisher("wf_box_mid",Image, queue_size = 10)
	#image_pub_right = rospy.Publisher("wf_box_right",Image, queue_size = 10)
	#object_pub = rospy.Publisher("critical_object_detected", String, queue_size = 10)
	rate = rospy.Rate(10)
	bridge = CvBridge()
	#frame_count = 0
	#prev_Y = 0
	#center = [0,0]
	# loop over the frames from the video stream
	while not rospy.is_shutdown():
		# grab the frame from the threaded video stream   and resize it
		# to have a maximum width of 400 pixels
		frame_left = vs1.read()
		frame_mid = vs2.read()
		# frame_right = vs3.read()
		frame_left = imutils.resize(frame_left, width=maxPixel)
		frame_mid =imutils.resize(frame_mid, width=maxPixel)
		#frame_right = imutils.resize(vs3.read(), width=maxPixel)

		# grab the frame dimensions and convert it to a blob
		(h1, w1) = frame_left.shape[:2]
		(h2, w2) = frame_mid.shape[:2]
		#(h3, w3) = frame_right.shape[:2]
		blob1 = cv2.dnn.blobFromImage(cv2.resize(frame_left, (300, 300)),
			0.007843, (300, 300), 127.5)
		blob2 = cv2.dnn.blobFromImage(cv2.resize(frame_mid, (300, 300)),
			0.007843, (300, 300), 127.5)
		# blob3 = cv2.dnn.blobFromImage(cv2.resize(frame_right, (300, 300)),
		# 	0.007843, (300, 300), 127.5)

		# pass the blob through the network and obtain the detections and
		# predictions
		net.setInput(blob1)
		detections_left = net.forward()
		net.setInput(blob2)
		detections_mid = net.forward()
		# net.setInput(blob3)
		# detection_right = net.forward()

		# loop over the detections
		for i in np.arange(0, detections_left.shape[2]):
			# extract the confidence (i.e., probability) associated with
			# the prediction
			confidence = detections_left[0, 0, i, 2]

			# filter out weak detections by ensuring the `confidence` is
			# greater than the minimum confidence
			if confidence > init_confidence:
				# extract the index of the class label from the
				# `detections`, then compute the (x, y)-coordinates of
				# the bounding box for the object
				idx = int(detections_left[0, 0, i, 1])
				print("index is: ", idx)
				box = detections_left[0, 0, i, 3:7] * np.array([w1, h1, w1, h1])
				(startX, startY, endX, endY) = box.astype("int")


				# draw the prediction on the frame
				label = "{}: {:.2f}%".format(CLASSES[idx],
					confidence * 100)
				cv2.rectangle(frame_left, (startX, startY), (endX, endY),
					COLORS[idx], 2)
				y = startY - 15 if startY - 15 > 15 else startY + 15
				cv2.putText(frame_left, label, (startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
		for i in np.arange(0, detections_mid.shape[2]):
			# extract the confidence (i.e., probability) associated with
			# the prediction
			confidence = detections_mid[0, 0, i, 2]

			# filter out weak detections by ensuring the `confidence` is
			# greater than the minimum confidence
			if confidence > init_confidence:
				# extract the index of the class label from the
				# `detections`, then compute the (x, y)-coordinates of
				# the bounding box for the object
				idx = int(detections_mid[0, 0, i, 1])
				box = detections_mid[0, 0, i, 3:7] * np.array([w2, h2, w2, h2])
				(startX, startY, endX, endY) = box.astype("int")


				# draw the prediction on the frame
				label = "{}: {:.2f}%".format(CLASSES[idx],
					confidence * 100)
				cv2.rectangle(frame_mid, (startX, startY), (endX, endY),
					COLORS[idx], 2)
				y = startY - 15 if startY - 15 > 15 else startY + 15
				cv2.putText(frame_mid, label, (startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
		# for i in np.arange(0, detections_right.shape[2]):
		# 	# extract the confidence (i.e., probability) associated with
		# 	# the prediction
		# 	confidence = detections_right[0, 0, i, 2]
		#
		# 	# filter out weak detections by ensuring the `confidence` is
		# 	# greater than the minimum confidence
		# 	if confidence > init_confidence:
		# 		# extract the index of the class label from the
		# 		# `detections`, then compute the (x, y)-coordinates of
		# 		# the bounding box for the object
		# 		idx = int(detections_right[0, 0, i, 1])
		# 		box = detections_right[0, 0, i, 3:7] * np.array([w3, h3, w3, h3])
		# 		(startX, startY, endX, endY) = box.astype("int")
		#
		# 		# draw the prediction on the frame
		# 		label = "{}: {:.2f}%".format(CLASSES[idx],
		# 			confidence * 100)
		# 		cv2.rectangle(frame_right, (startX, startY), (endX, endY),
		# 			COLORS[idx], 2)
		# 		y = startY - 15 if startY - 15 > 15 else startY + 15
		# 		cv2.putText(frame_right, label, (startX, y),
		# 			cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

		image_pub_left.publish(bridge.cv2_to_imgmsg(frame_left, "bgr8"))		# update the FPS counter
		image_pub_mid.publish(bridge.cv2_to_imgmsg(frame_mid, "bgr8"))
		# image_pub_right.publish(bridge.cv2_to_imgmsg(frame_right, "bgr8"))
		fps.update()


if __name__ == '__main__':
	CVControl()
# stop the timer and display FPS information
fps.stop()
# print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
# print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
