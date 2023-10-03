# Imports
import math
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

from djitellopy import tello
from time import sleep

from threading import Thread
import pygame

########### Drone Setup
me = tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()

########### Computer Vision Setups
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())

hsvRedBallDrone = {'hmin': 0, 'smin': 99, 'vmin': 124, 'hmax': 5, 'smax': 255, 'vmax': 212}
redVolleyBallDimLighting = {'hmin': 0, 'smin': 139, 'vmin': 137, 'hmax': 4, 'smax': 255, 'vmax': 255}

greenLower = (0, 139, 137)
greenUpper = (4, 255, 255)

pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    vs = VideoStream(src=0).start()

# vs = me.get_frame_read().frame
# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])
# allow the camera or video file to warm up
time.sleep(1.0)  # Reduce this time or delete entirely

# Coordinate Acquisition
coordArr = []


def newCoord(coordinates):
    coordArr.append(coordinates)


def getCoord():
    return coordArr


# Main Loop

############################################# Tracking ###############################################
def trackingFunc():
    # Variables required to construct trajectory line
    trackingLineX = 0
    trackingLineY = 0
    while True:

        frame = me.get_frame_read().frame  # Drone feed Assignment
        normalFrame = me.get_frame_read().frame
        # handle the frame from VideoCapture or VideoStream
        frame = frame[1] if args.get("video", False) else frame
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if frame is None:
            break
        # resize the frame, blur it, and convert it to the HSV
        # color space
        # Height 450, width 600
        frame = imutils.resize(frame, 600)  # Resize Here
        normalFrame = imutils.resize(normalFrame, 600)
        # Dimensions are Height 1200, Width 900
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)  # Original was hsv
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
            # only proceed if the radius meets a minimum size

            if radius > 3:  # Change Radius Range Here
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                #cv2.circle(frame, center, 5, (0, 0, 255), -1)

                newCoord((int(x), int(y), int(radius)))
                trackingLineX = int(x)
                trackingLineY = int(y)
                #print(int(radius))
                # Drawing line from center to ball

        if len(coordArr) > 0:
            cv2.line(frame, (300, 225), (trackingLineX, trackingLineY), (0,255,255), 2)
            cv2.line(frame, (300, 225), (trackingLineX, 225), (255, 0, 255), 1)

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
            #cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
        # show the frame to our screen

        ############################### USER INTERFACE STUFF #######################################

        # Drawing
        # Dimensions for drawing shapes:
        # Width 1200, Height 900


        global manualOverride
        global birdsEye
        # Safe zone rectangle
        cv2.rectangle(frame, (400, 250), (800, 650), (255, 0, 0), 5, cv2.FILLED, 1)  # Safe zone Rectangle
        #Counter zone rectangle
        cv2.rectangle(frame, (300, 250), (900, 650), (255, 255, 255), 2, cv2.FILLED, 1)

        ############### Mode / View Tracker

        if manualOverride:
            cv2.putText(frame, "Mode: Manual Override", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1,
                        cv2.LINE_AA)
        else:
            cv2.putText(frame, "Mode: CV Control", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1,
                        cv2.LINE_AA)

        if birdsEye:
            cv2.putText(frame, "View: Bird's Eye", (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 1,
                        cv2.LINE_AA)
        else:
            cv2.putText(frame, "View: Side", (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 1,
                        cv2.LINE_AA)

        if counter:
            cv2.putText(frame, "COUNTERING", (400, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 1,
                        cv2.LINE_AA)


        # Trajectory Line stuff
        global distance
        global horizontalDistance
        distance = 0
        if len(coordArr) > 0:
            cv2.line(frame, (300, 225), (trackingLineX, trackingLineY), (0,255,255), 2)
            distance = int(math.sqrt((300 - trackingLineX)**2 + (225 - trackingLineY)**2))
            horizontalDistance = abs(300 - trackingLineX)
            strDistance = str(distance)
            cv2.putText(frame, "Distance: " + strDistance, (int((300 + trackingLineX)/2), int((225 + trackingLineY) / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 1,
                        cv2.LINE_AA)

            cv2.putText(frame, "Hori Distance" + str(horizontalDistance),
                        (int((300 + trackingLineX) / 2), 225), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 0, 255), 1,
                        cv2.LINE_AA)
        # Forward, Up and Left will be green
        # Backward, down and right will be red
        # 0 will be Blue, Neutral
        # Put these in bottom right corner
        global netDepth
        global netVertical
        global netHorizontal

        if netHorizontal < 0:
            cv2.putText(frame, "LEFT", (430, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                        cv2.LINE_AA)
        elif netHorizontal > 0:
            cv2.putText(frame, "RIGHT", (430, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)

        if netVertical > 0:
            cv2.putText(frame, "UP", (430, 360), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                        cv2.LINE_AA)
        elif netVertical < 0:
            cv2.putText(frame, "DOWN", (430, 360), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)

        if netDepth < 0:
            cv2.putText(frame, "BACKWARD", (430, 420), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                        cv2.LINE_AA)
        elif netDepth > 0:
            cv2.putText(frame, "FORWARD", (430, 420), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                        cv2.LINE_AA)


        cv2.imshow("Frame", frame)
        # Below shows normal frame without the drawing
        #cv2.imshow("NormalFrame", normalFrame)

        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

    me.land()  # Lands the drone if loop ends for any reason

    # if we are not using a video file, stop the camera video stream
    if not args.get("video", False):
        vs.stop()
    # otherwise, release the camera
    else:
        vs.release()
    # close all windows
    cv2.destroyAllWindows()

########################################### Drone Control ############################################


########### Manual Control Setups

speed = 30

manualOverride = True
cvControl = False
switch = False
birdsEye = False

netHorizontal = 0
netVertical = 0
netDepth = 0
netRotation = 0
counter = False

distance = 0
horizontalDistance = 1234567

def droneControl():
    # pygame setups
    WIDTH, HEIGHT = 50, 50  # 900, 500
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Unnecessary")
    run = True
    # Dimensions for ball tracking:
    # Width 600, Height 450
    global birdsEye
    global netHorizontal
    global netDepth
    global netVertical
    global netRotation

    # Counter Setups
    horizontalTimeStart = time.time()
    timerStart = time.time()
    depthTime = time.time()
    pastHorizontal = 359

    last3X = []
    last3Y = []
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False


        # Manual Control
        global manualOverride
        global cvControl
        global switch
        global counter

        if len(coordArr) != 0:
            coordinates = getCoord()[len(coordArr) - 1]
        else:
            coordinates = (300, 225, 5)
        xVal = coordinates[0]
        yVal = coordinates[1]
        radius = coordinates[2]


        last3X.append(xVal)
        last3Y.append(yVal)
        backup = False
        if len(last3X) >= 3:
            x1 = xVal
            x2 = last3X[len(last3X) - 2]
            x3 = last3X[len(last3X) - 3]
            y1 = yVal
            y2 = last3Y[len(last3X) - 2]
            y3 = last3Y[len(last3X) - 3]
            print("x1: " + str(x1))
            print("x2: " + str(x2))
            print("x3: " + str(x3))
            if (x1 == x2 and x2 == x3) and (y1 == y2 and y2 == y3):
                backup = True

        print("Length of last3X: " + str(len(last3X)))





        keyPress = pygame.key.get_pressed()
        forward = 0
        backward = 0
        left = 0
        right = 0
        up = 0
        down = 0
        rotLeft = 0
        rotRight = 0


        # Manual Control
        if manualOverride:
            speed = 100

            if keyPress[pygame.K_w]:
                #print("forward")
                forward = speed

            elif keyPress[pygame.K_s]:
                #print("backward")
                backward = -speed

            if keyPress[pygame.K_a]:
                #print("left")
                left = -speed

            elif keyPress[pygame.K_d]:
                #print("right")
                right = speed

            if keyPress[pygame.K_q]:
                #print("up")
                up = -speed

            if keyPress[pygame.K_e]:
                #print("down")
                down = speed

            if keyPress[pygame.K_1]:
                #print("rotate left")
                rotLeft = -speed

            if keyPress[pygame.K_2]:
                #print("rotate right")
                rotRight = speed

            if keyPress[pygame.K_5]:
                print("land")
                me.land()
                run = False

            if keyPress[pygame.K_r]:
                me.send_rc_control(0, 0, 0, 0)

            if keyPress[pygame.K_t]:
                me.takeoff()

        # Tracking Ball
        elif cvControl:



            speed = 40
            if xVal < 200:  # xVal is to the left of the box
                #print("LEFT")

                left = -speed
            elif xVal > 400:  # xVal is to the right of the box
                #print("RIGHT")
                right = speed

            # yVal is not inside the box
            if yVal < 125:  # xVal is above the box
                if not birdsEye:
                    #print("UP")
                    up = speed
                else:
                    #print("FORWARD")
                    forward = -speed

            elif yVal > 325:
                if not birdsEye:
                    #print("DOWN")
                    down = -speed
                else:
                    #print("BACKWARD")
                    backward = speed

            radiusLower = 20
            radiusUpper = 50
            if radius < radiusLower:
                if not birdsEye:
                    print("FORWARD")
                    forward = 20
                else:
                    print("DOWN")
                    down = -speed
            elif radius > radiusUpper:
                if not birdsEye:
                    print("BACKWARD")
                    backward = -20
                else:
                    print("UP")
                    up = -speed

        if keyPress[pygame.K_0]:
            if manualOverride:
                manualOverride = False
                cvControl = True
            elif cvControl:
                manualOverride = True
                cvControl = False
            print("Switch")
            switch = True
            time.sleep(0.3)

        if keyPress[pygame.K_9]:
            if birdsEye:
                birdsEye = False
                print("Horizontal View")
            else:
                birdsEye = True
                print("Birds Eye View")
            time.sleep(0.3)



        netHorizontal = left + right
        netDepth = forward + backward
        netVertical = up + down
        netRotation = rotLeft + rotRight





        if switch:
            me.send_rc_control(0, 0, 0, 0)
            switch = False
        else:
            ######################################## COUNTER MEASURES ###########################################
            inCaseOfCounter = 0
            ########### KEEPING TIME
            timeSinceStartedMoving = round((time.time() - horizontalTimeStart), 2)
            # Everytime there's a new horizontal input it restarts the clock
            if cvControl:
                # Tries to kill the inertia, stop the swaying
                if pastHorizontal != netHorizontal and pastHorizontal != 359 and pastHorizontal != 0:
                    if timeSinceStartedMoving >= 2.0:
                        inCaseOfCounter = netHorizontal
                        if netHorizontal < 0:
                            netHorizontal = 100
                        elif netHorizontal > 0:
                            netHorizontal = -100
                        counter = True
                    # Everytime there's a new horizontal input it restarts the clock
                    horizontalTimeStart = time.time()

            # Need ^this for depth as well.

            if backup and cvControl:
                if not birdsEye:
                    me.send_rc_control(0, backward, 0, 0)
                else:
                    netVertical = up
                    me.send_rc_control(0, 0, up, 0)

            elif counter:
                me.send_rc_control(netHorizontal, netDepth, netVertical, netRotation)
                time.sleep(0.5)
                pastHorizontal = inCaseOfCounter
                counter = False
            else:
                me.send_rc_control(netHorizontal, netDepth, netVertical, netRotation)
                pastHorizontal = netHorizontal


            #################################### SAFETY ##############################################

            # Land because no inputs (glitches)
            if netHorizontal != 0 or netDepth != 0 or netVertical != 0 or netRotation != 0:
                timerStart = time.time()


            timeWithoutInstruction = int(time.time() - timerStart)

            if timeWithoutInstruction >= 15:
                print("Landing because No input within 10 seconds")
                me.land()

            if me.get_height() >= 200:
                me.land()



    me.land()


if __name__ == "__main__":
    Thread(target=trackingFunc).start()
    Thread(target=droneControl).start()
    me.takeoff()