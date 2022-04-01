import cv2
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
image = cv2.imread('HSVGreen2.png')
HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def getpos(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(HSV[y, x])


cv2.imshow("imageHSV", HSV)
cv2.imshow('image', image)
cv2.setMouseCallback("imageHSV", getpos)
cv2.waitKey(0)
# img = np.zeros((256,256,3),np.uint8)
#
# plt.imshow(img[:,:, ::-1])
#
# print(img[100,100])
# print(img[100,100,0])
# img[100,100]=(0,0,255)



