import numpy as np
import cv2
import os
import RPi.GPIO as GPIO
import time

HEIGHT = 240
WIDTH = 320

TRIG = 23
ECHO = 24
LED = 27
SWITCH = 18
BELT = 22

dir1A = 6
dir2A = 13
#spdA = 6
dir1B = 19
dir2B = 26
#spdB = 26

##################################

def move(x,y):
    if not GPIO.input(SWITCH) or x == -1 or y == -1:
        stop()
    elif (x<WIDTH*0.3):
        turnLeft()
    elif (x>WIDTH*0.7):
        turnRight()
    else:
        forward()

    if (y>HEIGHT*0.75 and not y== -1):
        GPIO.output(BELT,True)
    else:
        GPIO.output(BELT,False)
    return

def turnLeft():
    GPIO.output(dir1A, True)
    GPIO.output(dir2A, False)
    #GPIO.output(spdA, True)
    GPIO.output(dir1B, False)
    GPIO.output(dir2B, True)
    #GPIO.output(spdB, True)
    print "Left"
    return

def turnRight():
    GPIO.output(dir1A, False)
    GPIO.output(dir2A, True)
    #GPIO.output(spdA, True)
    GPIO.output(dir1B, True)
    GPIO.output(dir2B, False)
    #GPIO.output(spdB, True)
    print "Right"
    return

def forward():
    GPIO.output(dir1A, True)
    GPIO.output(dir2A, False)
    #GPIO.output(spdA, True)
    GPIO.output(dir1B, True)
    GPIO.output(dir2B, False)
    #GPIO.output(spdB, True)
    return

def reverse():
    GPIO.output(dir1A, False)
    GPIO.output(dir2A, True)
    #GPIO.output(spdA, True)
    GPIO.output(dir1B, True)
    GPIO.output(dir2B, False)
    #GPIO.output(spdB, True)
    return

def stop():
    GPIO.output(dir1A, False)
    GPIO.output(dir2A, False)
    #GPIO.output(spdA, False)
    GPIO.output(dir1B, False)
    GPIO.output(dir2B, False)
    #GPIO.output(spdB, False)
    return
###################################################################################################
def main():
    #GPIO.cleanup()
    GPIO.setmode(GPIO.BCM) #BOARD, BCM

    #Ultrasonic Sensor GPIO Setup
    
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    GPIO.output(TRIG, False)
    #print "Waiting for Sensor to Settle"
    #time.sleep(2)
    #GPIO.output(TRIG, True)
    #time.sleep(0.00001)
    #GPIO.output(TRIG, False)
    #while GPIO.input(ECHO)==0:
    #    pulse_start = time.time()
    #while GPIO.input(ECHO)==1:
    #    pulse_end = time.time()
    #pulse_duration = pulse_end - pulse_start
    #distance = pulse_duration*17150
    #distance = round(distance, 2)
    #print "Distance:",distance,"cm"
    
    GPIO.setup(SWITCH,GPIO.IN)
    GPIO.setup(17,GPIO.OUT)
    GPIO.output(17,True)
    GPIO.setup(LED,GPIO.OUT)
    GPIO.output(LED,True)
    GPIO.setup(BELT,GPIO.OUT)
    GPIO.output(BELT,False)
        
    #MOTOR GPIO Setup
    GPIO.setup(dir1A,GPIO.OUT)
    GPIO.setup(dir2A,GPIO.OUT)
    #GPIO.setup(spdA,GPIO.OUT)
    GPIO.setup(dir1B,GPIO.OUT)
    GPIO.setup(dir2B,GPIO.OUT)
    #GPIO.setup(spdB,GPIO.OUT)
    GPIO.output(dir1A, False)
    GPIO.output(dir2A, False)
    #GPIO.output(spdA, False)
    GPIO.output(dir1B, False)
    GPIO.output(dir2B, False)
    #GPIO.output(spdB, False)
    
    
    #imgOriginal = cv2.imread("image2.jpg")               # open image
    capWebcam = cv2.VideoCapture(0)                     # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

                                                        # show original resolution
    print "default resolution = " + str(capWebcam.get(cv2.CAP_PROP_FRAME_WIDTH)) + "x" + str(capWebcam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    capWebcam.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)              # change resolution to 320x240 for faster processing
    capWebcam.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

                                                        # show updated resolution
    print "updated resolution = " + str(capWebcam.get(cv2.CAP_PROP_FRAME_WIDTH)) + "x" + str(capWebcam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if capWebcam.isOpened() == False:                           # check if VideoCapture object was associated to webcam successfully
        print "error: capWebcam not accessed successfully\n\n"          # if not, print error message to std out
        os.system("pause")                                              # pause until user presses a key so user can see error message
        return                                                          # and exit function (which exits program)
    # end if

    while cv2.waitKey(1) != 27 and capWebcam.isOpened():                # until the Esc key is pressed or webcam connection is lost
        blnFrameReadSuccessfully, imgOriginal = capWebcam.read()            # read next frame

        if not blnFrameReadSuccessfully or imgOriginal is None:             # if frame was not read successfully
            print "error: frame not read from webcam\n"                     # print error message to std out
            os.system("pause")                                              # pause until user presses a key so user can see error message
            break                                                           # exit while loop (which exits program)
        # end if
    
        if imgOriginal is None:                             # if image was not read successfully
            print "error: image not read from file \n\n"        # print error message to std out
            os.system("pause")                                  # pause so user can see error message
            return                                              # and exit function (which exits program)
        # end if

        imgHSV = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2HSV)

        imgThreshWhite = cv2.inRange(imgHSV, np.array([0, 0, 135]), np.array([360, 51, 255]))
        imgThreshBall = cv2.inRange(imgHSV, np.array([30, 65, 70]), np.array([40, 255, 255]))

        imgThreshWhite = cv2.GaussianBlur(imgThreshWhite, (3, 3), 2)

        imgThreshWhite = cv2.dilate(imgThreshWhite, np.ones((5,5),np.uint8))
        imgThreshWhite = cv2.erode(imgThreshWhite, np.ones((5,5),np.uint8))
        whitePixels = cv2.findNonZero(imgThreshWhite)
##
##        xsum=0
##        ysum=0
##        count=0
##        if np.any(whitePixels):
##            print whitePixels[0]
##            print whitePixels[:,0,0].size
##            for i in whitePixels[:,0,0]:
##                if i<whitePixels[:,0,0].size:
##                    xsum=xsum+whitePixels[i,0,0]
##                    ysum=ysum+whitePixels[i,0,1]
##                    count=count+1
##        if count!=0:
##            print xsum/count
##            print ysum/count
##            print " "
            
##        edges = cv2.Canny(imgThreshWhite, 75, 100, apertureSize = 3)
##        print imgThreshWhite.shape[1]
##        print imgThreshWhite.shape
##        minLineLength = imgThreshWhite.shape[1]-300
##        lines = cv2.HoughLinesP(image=edges, rho=0.02, theta=np.pi/500, threshold=20, lines=np.array([]), minLineLength=minLineLength, maxLineGap=100)
##        a,b,c = lines.shape
##        for i in range(a):
##            cv2.line(imgOriginal, (lines[i][0][0],lines[i][0][1]), (lines[i][0][2],lines[i][0][3]), (0,0,255), 2, cv2.LINE_AA)
##        cv2.imshow('edges', edges)
##        cv2.imshow('houghlines',imgOriginal)
                
        imgThreshBall = cv2.GaussianBlur(imgThreshBall, (3, 3), 2)
        imgThreshBall = cv2.dilate(imgThreshBall, np.ones((5,5),np.uint8))
        imgThreshBall = cv2.erode(imgThreshBall, np.ones((5,5),np.uint8))
        ballPixels = cv2.findNonZero(imgThreshBall)

        ballxsum=0
        ballysum=0
        ballcount=0
        if np.any(ballPixels):
            print ballPixels[0]
            print ballPixels[:,0,0].size
            for i in ballPixels[:,0,0]:
                if i<ballPixels[:,0,0].size:
                    ballxsum=ballxsum+ballPixels[i,0,0]
                    ballysum=ballysum+ballPixels[i,0,1]
                    ballcount=ballcount+1
        if ballcount!=0:
            ballavgx=ballxsum/ballcount
            ballavgy=ballysum/ballcount
            print ballavgx
            print ballavgy
            print " "    

            move(ballavgx,ballavgy)

        else:
            move(-1,-1)
    #    cv2.imwrite('imageBall.jpeg', imgThreshBall)
        
    ##    imgThresh = cv2.add(imgThreshWhite, imgThreshBall)
    ##
    ##    imgThresh = cv2.GaussianBlur(imgThresh, (3, 3), 2)
    ##
    ##    imgThresh = cv2.dilate(imgThresh, np.ones((5,5),np.uint8))
    ##    imgThresh = cv2.erode(imgThresh, np.ones((5,5),np.uint8))

    ##    intRows, intColumns = imgThresh.shape

    ##    circles = cv2.HoughCircles(imgThresh, cv2.HOUGH_GRADIENT, 5, intRows / 4)      # fill variable circles with all circles in the processed image

    ##    if circles is not None:                     # this line is necessary to keep program from crashing on next line if no circles were found
    ##        for circle in circles[0]:                           # for each circle
    ##            x, y, radius = circle                                                                       # break out x, y, and radius
    ##            print "ball position x = " + str(x) + ", y = " + str(y) + ", radius = " + str(radius)       # print ball position and radius
    ##            cv2.circle(imgOriginal, (x, y), 3, (0, 255, 0), -1)           # draw small green circle at center of detected object
    ##            cv2.circle(imgOriginal, (x, y), radius, (0, 0, 255), 3)                     # draw red circle around the detected object
            # end for
        # end if

        cv2.namedWindow("imgOriginal", cv2.WINDOW_NORMAL)            # create windows, use WINDOW_AUTOSIZE for a fixed window size
    ##    cv2.namedWindow("imgThresh", cv2.WINDOW_NORMAL)           # or use WINDOW_NORMAL to allow window resizing
        cv2.namedWindow("imgHSV", cv2.WINDOW_NORMAL)
        cv2.namedWindow("imgThreshWhite", cv2.WINDOW_NORMAL)
        cv2.namedWindow("imgThreshBall", cv2.WINDOW_NORMAL)
        
        cv2.imshow("imgOriginal", imgOriginal)                 # show windows
    ##  cv2.imshow("imgThresh", imgThresh)
        cv2.imshow("imgHSV", imgHSV)
        cv2.imshow("imgThreshWhite", imgThreshWhite)
        cv2.imshow("imgThreshBall", imgThreshBall)

    #cv2.waitKey()                               # hold windows open until user presses a key

    cv2.destroyAllWindows()                     # remove windows from memory
    GPIO.cleanup()
    
    return

###################################################################################################
if __name__ == "__main__":
    main()
