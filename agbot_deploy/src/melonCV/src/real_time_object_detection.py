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
#maxPixel = rospy.get_param("/maxPixel")
maxPixel = 300
vs = VideoStream(src=0).start()
objectDetected = False


def CVControl():
	rspkg = rospkg.RosPack()
 	#prototxt_path = os.path.join(rspkg.get_path('melonCV'),"src","MNSSD_melon_1200000.prototxt")
 	#caffemodel_path = os.path.join(rspkg.get_path('melonCV'),"src","MNSSD_melon_1200000.caffemodel")
	prototxt_path = "~/agtrec/agbot_deploy/src/melonCV/src/NSSD_melon_1200000.prototxt"
	caffemodel_path = "~/agtrec/agbot_deploy/src/melonCV/src/MNSSD_melon_1200000.caffemodel"
	init_confidence = 0.2
	# initialize the list of class labels MobileNet SSD was trained to
	# detect, then generate a set of bounding box colors for each class
	CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
		"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
		"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
		"sofa", "train", "tvmonitor", "watermelon"]
	COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

	# load our serialized model from disk
	print("[INFO] loading model...")
	net = cv2.dnn.readNetFromCaffe(prototxt_path, caffemodel_path)

	# initialize the video stream, allow the cammera sensor to warmup,
	# and initialize the FPS counter
	print("[INFO] starting video stream...")

	time.sleep(2.0)
	fps = FPS().start()




	rospy.init_node("real_time_object_detection", anonymous = True)
	image_pub = rospy.Publisher("watermelon_cv",Image, queue_size = 10)
	location_pub = rospy.Publisher("watermelon_location", Int16MultiArray, queue_size = 10)
	melon_pub = rospy.Publisher("melon_detected", Bool, queue_size = 10)
	object_pub = rospy.Publisher("critical_object_detected", String, queue_size = 10)
	rate = rospy.Rate(10)
	bridge = CvBridge()
	frame_count = 0
	prev_Y = 0
	center = [0,0]
	# loop over the frames from the video stream
	while not rospy.is_shutdown():
		# grab the frame from the threaded video stream and resize it
		# to have a maximum width of 400 pixels
		frame = vs.read()
		frame = imutils.resize(frame, width=maxPixel)

		# grab the frame dimensions and convert it to a blob
		(h, w) = frame.shape[:2]
		blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
			0.007843, (300, 300), 127.5)

		# pass the blob through the network and obtain the detections and
		# predictions
		net.setInput(blob)
		detections = net.forward()
		objectDetected = False
		# loop over the detections
		for i in np.arange(0, detections.shape[2]):
			# extract the confidence (i.e., probability) associated with
			# the prediction
			confidence = detections[0, 0, i, 2]

			# filter out weak detections by ensuring the `confidence` is
			# greater than the minimum confidence
			if confidence > init_confidence:
				# extract the index of the class label from the
				# `detections`, then compute the (x, y)-coordinates of
				# the bounding box for the object
				idx = int(detections[0, 0, i, 1])
				box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")


				# draw the prediction on the frame
				label = "{}: {:.2f}%".format(CLASSES[idx],
					confidence * 100)
				cv2.rectangle(frame, (startX, startY), (endX, endY),
					COLORS[idx], 2)
				y = startY - 15 if startY - 15 > 15 else startY + 15
				cv2.putText(frame, label, (startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
				size = (endX - startX)*(endY-startY)
				if idx == 21:
					centerX = (endX - startX)/2 + startX
					centerY = (endY - startY)/2 + startY
					if centerY > prev_Y: 						#only take the closest melon
						center[0] = centerX
						center[1] = centerY
						prev_Y = centerY
				if idx == 15 or idx == 7 or idx == 10 or idx == 12 or idx == 13 or idx == 14 or idx == 17:
					if size > 150*150:
						objectDetected = True
						#object_pub.publish(CLASSES[idx])
					else:
						objectDetected = False
				else:
					objectDetected = False
						#object_pub.publish("None")
		if objectDetected:
			object_pub.publish("Detected")
		else:
			object_pub.publish("None")
		#output result every 10 frames
		frame_count += 1
		if frame_count == 10:
			if not center[0] == 0:
				melon_pub.publish(True)

			else:
				melon_pub.publish(False)
			location_pub.publish(Int16MultiArray(data=center))
			prev_Y = 0
			center[0] = 0
			center[1] = 0
			frame_count = 0
		# show the output frame
		#cv2.imshow("Frame", frame)
		#key = cv2.waitKey(1) & 0xFF

		# if the `q` key was pressed, break from the loop
		#if key == ord("q"):
		#	break
		image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))		# update the FPS counter
		#fps.update()


if __name__ == '__main__':
	CVControl()
# stop the timer and display FPS information
# fps.stop()
# print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
# print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
