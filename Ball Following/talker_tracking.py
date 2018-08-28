from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import time
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64

def runVision():
	# Defining the lower and upper boundaries of the "blue"
	# ball in the HSV color space, then initialising the
	# list of tracked points
	blueLower = (48, 62, 88)
	blueUpper = (151, 238, 255)
	pts = deque(maxlen=10)
	count = 0
	camera = cv2.VideoCapture(0)

	# Initialising background subtraction - motion detection
	fgbg = cv2.createBackgroundSubtractorMOG2()

	while True:
		# Grabbing the current frame
		(grabbed, frame) = camera.read()

		# Resizing the frame
		frame = imutils.resize(frame, width=720)

		# Applying background subtraction - motion detection
		mask = fgbg.apply(frame)
		res = cv2.bitwise_and(frame, frame, mask = mask)

	    # Blurring the frame and converting it to the HSV
		blurred = cv2.GaussianBlur(res, (11, 11), 0)
		hsv = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)

		# Constructing a mask for the color "blue", then performing
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, blueLower, blueUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# Finding contours in the mask and initialising the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None

		# If at least one contour was found
		if len(cnts) > 0:
			# Finding the largest contour in the mask, then using
			# it to compute the minimum enclosing circle and centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# If the radius meets a minimum size
			if radius > 10:
				# Drawing the circle and centroid on the frame,
				# then updating the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)

		# Updating the points queue
		pts.appendleft(center)

		# If the ball is being tracked - there is an element at the beggining
		# of the tracking deque
		if pts[0] != None:
			y0 = pts[0][1]
			x0 = pts[0][0]

			# Initialising node
			rospy.init_node('talkertracking', anonymous=True)

			# Sending coordinates to subscriber node
			pub = rospy.Publisher('centery', Float64, queue_size=10)
			pub2 = rospy.Publisher('centerx', Float64, queue_size=10)
			pub.publish(y0)
			pub2.publish(x0)

			# Printing the coordinates in the terminal
			rospy.loginfo(x0)
			rospy.loginfo(y0)

		# Looping over the set of tracked points
		for i in xrange(1, len(pts)):
			# Ignoring tracked points equal to None
			if pts[i - 1] is None or pts[i] is None:
				continue

			# Computing the thickness of the line and drawing the connecting lines
			thickness = int(np.sqrt(10 / float(i + 1)) * 2.5)
			cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

		# Showing coordinates of the ball on the screen
		cv2.putText(frame, "x, y: {}".format(pts[0]),
			(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
			0.35, (0, 0, 255), 1)

		# Showing the frame on the screen
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF

		# If the 'q' key is pressed, stopping the loop
		if key == ord("q"):
			break

	# Cleaning up the camera and closing any open windows
	camera.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
	try:
		runVision()
	except rospy.ROSInterruptException:
		pass
