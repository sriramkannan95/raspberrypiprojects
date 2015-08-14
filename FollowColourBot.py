# FollowColourBot.py - track a colourful object with a webcam mounted atop pan-and-tilt servo and follow it (haven't implemented forward/backward yet)  with help of base DC motors
# Running on a Raspberry Pi, Raspbian Wheezy OS
# (c) Sriram Kannan, 2015

import cv2 # opencv has to be installed in the Raspberry Pi
import numpy as np
from RPIO import PWM
import RPi.GPIO as GPIO # always needed with RPi.GPIO  
import time

GPIO.setmode(GPIO.BCM)


GPIO.setup(26, GPIO.OUT)
p = GPIO.PWM(26, 50) #left two wheels in chasis - forward
GPIO.setup(19, GPIO.OUT)
q = GPIO.PWM(19, 50) #right two wheels in chasis - forward
GPIO.setup(6, GPIO.OUT)
r = GPIO.PWM(6, 50) #right two wheels in chasis - backward
GPIO.setup(13, GPIO.OUT)
s = GPIO.PWM(13, 50) #left two wheels in chasis - backward




# the HSV range we use to detect the colourful object
Hmin = 159
Hmax = 179 
Smin = 108
Smax = 255
Vmin = 80
Vmax = 255
# minimum detected area
minArea = 10
# frame parameters
width = 160
height = 120
cx = int(0.5 * width)
cy = int(0.5 * height)
# servo PWM pulsewidths
dServo = 20
servoValx = 1500
servoValy = 1500
servoMin = 500
servoMax = 2500
# the maximum delta (pixels) when the object is still considered centered
eps = 15

rangeMin = np.array([Hmin, Smin, Vmin], np.uint8)
rangeMax = np.array([Hmax, Smax, Vmax], np.uint8)

cv2.namedWindow("Adjustment")
cv2.namedWindow("Video")
cv2.namedWindow("Binary")

def nothing(*args):
  pass

def updateRanges():
  Hmin = cv2.getTrackbarPos("Hmin", "Adjustment")
  Hmax = cv2.getTrackbarPos("Hmax", "Adjustment")
  Smin = cv2.getTrackbarPos("Smin", "Adjustment")
  Smax = cv2.getTrackbarPos("Smax", "Adjustment")
  Vmin = cv2.getTrackbarPos("Vmin", "Adjustment")
  Vmax = cv2.getTrackbarPos("Vmax", "Adjustment")
  minArea = cv2.getTrackbarPos("minArea", "Adjustment")
  rangeMin = np.array([Hmin, Smin, Vmin], np.uint8)
  rangeMax = np.array([Hmax, Smax, Vmax], np.uint8)
  return rangeMin, rangeMax, minArea

# create controls for adjusting the detection range
cv2.createTrackbar("Hmin", "Adjustment", Hmin, 179, nothing)
cv2.createTrackbar("Hmax", "Adjustment", Hmax, 179, nothing)
cv2.createTrackbar("Smin", "Adjustment", Smin, 255, nothing)
cv2.createTrackbar("Smax", "Adjustment", Smax, 255, nothing)
cv2.createTrackbar("Vmin", "Adjustment", Vmin, 255, nothing)
cv2.createTrackbar("Vmax", "Adjustment", Vmax, 255, nothing)
cv2.createTrackbar("minArea", "Adjustment", minArea, 1000, nothing)
# capture from the first webcam found
cap = cv2.VideoCapture(0)

if cap.isOpened():
  cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)

  servo = PWM.Servo()
  servo.set_servo(17, servoValy) # tilt servo pin 17
  servo.set_servo(5, servoValx) # pan servo pin 17

  p.start(0)
  q.start(0)
  r.start(0)
  s.start(0)            

  while True:
    servo.set_servo(17, servoValy)
    servo.set_servo(5, servoValx)
 
    # read a frame
    ret, image = cap.read()
    # blur the frame
    image = cv2.blur(image, (3, 3))
    # convert to HSV
    imgHSV = cv2.cvtColor(image, cv2.cv.CV_BGR2HSV)
    rangeMin, rangeMax, minArea = updateRanges()
    # pixels within range are set to 1, others to 0
    imgThresh = cv2.inRange(imgHSV, rangeMin, rangeMax)
    #imgThresh = cv2.blur(imgThresh, (2, 2))

    # calculate image moments
    moments = cv2.moments(imgThresh, True)
    if moments['m00'] >= minArea:
      # calculate the centroid of the object using the moments
      x = moments['m10'] / moments['m00']
      y = moments['m01'] / moments['m00']
      cv2.circle(image, (int(x), int(y)), 5, (0, 255, 0), -1)
      # move the servo if necessary, check limits
      dx = x - cx
      dy = y - cy
      stepx = abs(int(dx / 20) * dServo)
      stepy = abs(int(dy / 20) * dServo)
      if (x - cx) > eps:
        servoValx -= stepx
      elif (cx - x) > eps:
        servoValx += stepx
      if servoValx <= servoMin:
        servoValx = servoMin
        r.ChangeDutyCycle(70)
        p.ChangeDutyCycle(70)
        time.sleep(0.5)
        r.ChangeDutyCycle(0)
        p.ChangeDutyCycle(0)
      if servoValx >= servoMax:
        servoValx = servoMax
        s.ChangeDutyCycle(70)
        q.ChangeDutyCycle(70)
        time.sleep(0.5)
        s.ChangeDutyCycle(0)
        q.ChangeDutyCycle(0)

      if (y - cy) > eps:
        servoValy += stepy
      elif (cy - y) > eps:
        servoValy -= stepy
      if servoValy <= 1000:
        servoValy = 1000
      if servoValy >= 2000:
        servoValy = 2000

    cv2.imshow("Video", image)
    cv2.imshow("Binary", imgThresh)
    key = cv2.waitKey(10) 
    if key == 27:
      break

p.stop()
q.stop()
r.stop()
s.stop()



cv2.destroyAllWindows()
