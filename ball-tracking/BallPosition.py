from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

class BallPosition:
    def __init__(self):
        self.vs = VideoStream(2).start()
        # allow the camera or video file to warm up
        time.sleep(2.0)
        self.Center = np.array([353.,242.])
        self.LtR = 292
        self.BtT = 292
        self.length = 0.1143

    def Position(self, color):
        # self.x = 0.
        # self.y = 0.
        # position = np.array([self.x,self.y])

        if color == "green":
            # # Green ball (H: 0 - 180, S: 0 - 255, V: 0 - 255)
            Lower = (29, 86, 6)
            Upper = (64, 255, 255)
            print("Green")
        elif color == "orange":
            # Orange pingpong ball
            Lower = (0, 91, 132)
            Upper = (15, 255, 255)
            print("orange")
        elif color == "white":
            # White ball
            Lower = (0, 0, 60)
            Upper = (0, 0, 100)
            print("white")
        else:
            print("Please specify color")
            return position


        frame = self.vs.read()

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        pts = deque(maxlen=64)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, Lower, Upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print("Center:", center)
            # update the points queue
            pts.appendleft(center)
        # loop over the set of tracked points
        for i in range(1, len(pts)):
            # if either of the tracked points are None, ignore
            # them
            if pts[i - 1] is None or pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        if color == "green":
            self.Center = center
        elif color == "orange":
            x = (self.Center[0]-center[0])*self.length/self.LtR
            y = (self.Center[1]-center[1])*self.length/self.BtT
            center = (x,y)
        return center

    def shutdown(self):
        # if we are not using a video file, stop the camera video stream
        self.vs.stop()
        # close all windows
        cv2.destroyAllWindows()
