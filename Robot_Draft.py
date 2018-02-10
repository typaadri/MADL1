import cv2
import numpy as np
import os
import RPi.GPIO as GPIO
import time

###################################################################################################
def main():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM) #BOARD, BCM

    TRIG = 23
    ECHO = 24

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    GPIO.output(TRIG, False)
    print "Waiting for Sensor to Settle"
    time.sleep(2)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration*17150
    distance = round(distance, 2)
    print "Distance:",distance,"cm"
    
    GPIO.setup(18,GPIO.OUT)
    GPIO.output(18,True)
    
    imgOriginal = cv2.imread("image2.jpg")               # open image

    if imgOriginal is None:                             # if image was not read successfully
        print "error: image not read from file \n\n"        # print error message to std out
        os.system("pause")                                  # pause so user can see error message
        return                                              # and exit function (which exits program)
    # end if

    imgHSV = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2HSV)

    imgThreshWhite = cv2.inRange(imgHSV, np.array([0, 0, 135]), np.array([360, 51, 255]))
    imgThreshBall = cv2.inRange(imgHSV, np.array([30, 135, 135]), np.array([40, 255, 255]))

    imgThreshWhite = cv2.GaussianBlur(imgThreshWhite, (3, 3), 2)

    imgThreshWhite = cv2.dilate(imgThreshWhite, np.ones((5,5),np.uint8))
    imgThreshWhite = cv2.erode(imgThreshWhite, np.ones((5,5),np.uint8))
    
    imgThreshBall = cv2.GaussianBlur(imgThreshBall, (3, 3), 2)

    imgThreshBall = cv2.dilate(imgThreshBall, np.ones((5,5),np.uint8))
    imgThreshBall = cv2.erode(imgThreshBall, np.ones((5,5),np.uint8))
    
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

    #cv2.namedWindow("imgOriginal", cv2.WINDOW_NORMAL)            # create windows, use WINDOW_AUTOSIZE for a fixed window size
##    cv2.namedWindow("imgThresh", cv2.WINDOW_NORMAL)           # or use WINDOW_NORMAL to allow window resizing
    #cv2.namedWindow("imgHSV", cv2.WINDOW_NORMAL)
    #cv2.namedWindow("imgThreshWhite", cv2.WINDOW_NORMAL)
    #cv2.namedWindow("imgThreshBall", cv2.WINDOW_NORMAL)
    
    #cv2.imshow("imgOriginal", imgOriginal)                 # show windows
##    cv2.imshow("imgThresh", imgThresh)
    #cv2.imshow("imgHSV", imgHSV)
    #cv2.imshow("imgThreshWhite", imgThreshWhite)
    #cv2.imshow("imgThreshBall", imgThreshBall)

    cv2.waitKey()                               # hold windows open until user presses a key

    cv2.destroyAllWindows()                     # remove windows from memory
    GPIO.cleanup()
    
    return

###################################################################################################
if __name__ == "__main__":
    main()
