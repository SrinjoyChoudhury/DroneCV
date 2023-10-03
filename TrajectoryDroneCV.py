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
redVolleyBallDimLighting = {'hmin': 0, 'smin': 132, 'vmin': 89, 'hmax': 179, 'smax': 255, 'vmax': 255}

{'hmin': 0, 'smin': 159, 'vmin': 0, 'hmax': 7, 'smax': 255, 'vmax': 255}
greenLower = (0, 159, 0)
greenUpper = (7, 255, 255)

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

coordInfoArr = []

def newCoord(coordinates):
    coordArr.append(coordinates)


def getCoord():
    return coordArr

def newTrajCoord(newX, newY):
    coordInfoArr.append((newX, newY))

def getTrajCoord():
    return coordInfoArr

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

            if radius > 1:  # Change Radius Range Here
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
            #cv2.line(frame, (300, 225), (trackingLineX, 225), (255, 0, 255), 1)

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


        # Center to Ball Line stuff
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

            # cv2.putText(frame, "Hori Distance" + str(horizontalDistance),
            #             (int((300 + trackingLineX) / 2), 225), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            #             (255, 0, 255), 1,
            #             cv2.LINE_AA)


        # Ball Ahead Trajectory
        lag = 70
        if len(coordArr) >= lag:
            oldCoords = getCoord()[len(coordArr) - lag]
            oldX = oldCoords[0]
            oldY = oldCoords[1]


            # Lag Trajectory Line
            # pygame.draw.line(WIN, (0, 255, 255), (oldX, oldY), (x, y), 5)

            ############### Forward Trajectory
            xDifference = trackingLineX - oldX
            yDifference = trackingLineY - oldY

            # if oldX < trackingLineX:
            #     xDifference = trackingLineX - oldX
            #     newX = trackingLineX + xDifference
            # else:
            #     xDifference = oldX - trackingLineX
            #     newX = trackingLineX - xDifference
            #
            # if oldY < trackingLineY:
            #     yDifference = trackingLineY - oldY
            #     newY = trackingLineY + yDifference
            # else:
            #     yDifference = oldY - trackingLineY
            #     newY = trackingLineY - yDifference

            if oldX < trackingLineX:
                newX = trackingLineX + (trackingLineX - oldX)
            else:
                newX = trackingLineX - (oldX - trackingLineX)
            if oldY < trackingLineY:
                newY = trackingLineY + (trackingLineY - oldY)
            else:
                newY = trackingLineY - (oldY - trackingLineY)



            if abs(xDifference) > 25 or abs(yDifference) > 25:
                #arrowOvershootX = newX - 50
                #arrowOvershootY = newY - 50
                cv2.arrowedLine(frame, (trackingLineX, trackingLineY), (newX, newY), (0, 255, 0), 10)
                #newTrajCoord(newX, newY)
                newTrajCoord(xDifference, yDifference)


        # Current Movement

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

speed = 100

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
    depthTimeStart = time.time()
    masterTime = time.time()
    pastHorizontal = 359
    pastDepth = 259

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
            cvControl = False
            coordinates = (300, 225, 5)
        xVal = coordinates[0]
        yVal = coordinates[1]
        radius = coordinates[2]

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

            # if keyPress[pygame.K_t]:
            #     me.takeoff()


        ############################################ TRAJECTORY MAPPING ############################################
        # Tracking Ball
        elif cvControl and len(getTrajCoord()) > 0:
            # You need xDifference yDifference because you want it to not move at all if trajectory distance is
            # small enough. Which is why you need your distance values
            # Rewrite your getter and setter up top to give u the x y differences
            # The problem is the x y differencecs are not representation of direction because u made em
            # that way. Find a way around that first
            speed = 100
            trajXValue = getTrajCoord()[0]
            trajYValue = getTrajCoord()[1]


            # Preserve the signage of the differences, and maintain trajectory arrow
            x = abs(-15)
            print(x)
            if abs(trajXValue) > 25 or abs(trajYValue) > 25:

                if trajXValue < 0:
                    left = -speed
                else:
                    right = speed

                if trajYValue < 0:
                    up = speed
                else:
                    down = -speed



            #if xDifference < 0:
            #    left =
            # Here, based on what trajectory mapping tells us, we instruct the drone

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

            # Commenting out drone instruction code
            #me.send_rc_control(netHorizontal, netDepth, netVertical, netRotation)
            print("NetHorizontal: ")
            print(netHorizontal)
            print(" NetDepth: ")
            print(netDepth)
            print(" netVertical:")
            print(netVertical)
            print(" NetRotation: ")
            print(netRotation)
            pastHorizontal = netHorizontal
            pastDepth = netDepth


        #################################### SAFETY ##############################################

        # Land because no inputs (glitches)
        if netHorizontal != 0 or netDepth != 0 or netVertical != 0 or netRotation != 0:
            timerStart = time.time()

        timeWithoutInstruction = int(time.time() - timerStart)
        if timeWithoutInstruction >= 15:
            print("Landing because No input within 10 seconds")
            #me.land()

        if me.get_height() >= 250:
            print("Landed due to height")
            me.land()

        # Fly time 40 seconds
        if (int(time.time() - masterTime)) >= 180:
            print("3 minutes over")
            #me.land()

        print("MasterTimer: " + str(int(time.time() - masterTime)))



    me.land()


if __name__ == "__main__":
    Thread(target=trackingFunc).start()
    Thread(target=droneControl).start()
    #me.takeoff()