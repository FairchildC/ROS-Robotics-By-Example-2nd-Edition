#!/usr/bin/env python
import cv2
import numpy

# read png image and convert the image to HSV
image = cv2.imread("/home/carol/Desktop/mission_scene.png", cv2.IMREAD_COLOR)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  

# find green objects in the image
lower_green = numpy.array([68, 47, 182], numpy.uint8)
upper_green = numpy.array([88, 147, 255], numpy.uint8)
mask = cv2.inRange(hsv, lower_green, upper_green)

cv2.imwrite("hsv_mask2.png", mask)

