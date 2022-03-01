# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import requests;
from bs4 import BeautifulSoup;
import numpy as np
import cv2
import imutils
import matplotlib.pyplot as plt

# print(cv.__version__)
# img = cv.imread("E:\SUSTECH\Student Work\FORSTUDY\Lab\IEEE SoftRobot Competition\ComputerVersion\GreenBlocks.jpg",1)
# cv.imshow("1",img)
# cv.waitKey()
from cv2 import VideoCapture

ball_color = 'red'  # choose color to recognize
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 46]), 'Upper': np.array([77, 255, 255])},
              }  # define the exact bound for each color

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # start video capture
cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)  # open a window to show
# print("1")

KNOWN_DISTANCE = 15
KNOWN_WIDTH = 0.4724
KNOWN_HEIGHT = 1.5748


def find_marker(image):
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
    edged_img = cv2.Canny(gray_img, 35, 125)

    (cnts, _) = cv2.findContours(edged_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)

    return cv2.minAreaRect(c)


# IMAGE_PATH = ["Picture1.jpg", "Picture2.jpg"]
focalLength = 2.8


# image = cv2.imread(IMAGE_PATH[0])
# marker = find_marker(image)
# focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH


def distance_to_camera(knownWidth, focalLength, perWidth):
    # if perWidth != 0:
    return (knownWidth * focalLength) / perWidth


# return 0

def calculate_focalDistance(img_path):
    first_image = cv2.imread(img_path)

    marker = find_marker(first_image)

    focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

    print('focalLength = ', focalLength)

    return focalLength


img_path = "Picture1.jpg"
focalLength = calculate_focalDistance(img_path)

#
while cap.isOpened():  # while the capture is open
    # print("1")
    ret, frame = cap.read()  # read ret and frame
    if ret:
        # print("2")
        if frame is not None:  # have image
            # print("3")
            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # using GaussianBlur
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # From BGR to HSV
            erode_hsv = cv2.erode(hsv, None, iterations=2)  # erode to reduce noise
            inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
            # delete backgrounds
            cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            # find contours

            # marker = find_marker(frame)

            # if len(cnts) > 1:
            # cnts = cnts[0] if imutils.is_cv2() else cnts[1]
            # cntsSorted = sorted(cnts,key=lambda x: cv2.contourArea(x))

            # def cnt_area(cnt):
            #     area = cv2.contourArea(cnt)
            #     return area
            #
            #
            # contours.sort_contours()

            # print(len(cnts))
            if len(cnts) != 0:
                # counter = 0

                # cnts.sort(key=cv2.contourArea(cnts), reverse=False)

                Max = max(cnts, key=cv2.contourArea)  # find outer edges of the rectangle
                rect = cv2.minAreaRect(Max)  # draw the min area rectangle
                box = cv2.boxPoints(rect)  # save the corner point to box
                if rect[1][0] != 0:
                    if rect[1][1] < rect[1][0]:
                        inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect[1][0])
                        cv2.putText(frame, "%.2fcm" % (inches * 2.54), (frame.shape[1] - 200, frame.shape[0] - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
                    else:
                        inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect[1][1])
                        cv2.putText(frame, "%.2fcm" % (inches * 2.54), (frame.shape[1] - 200, frame.shape[0] - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)

                cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)
                if len(cnts) > 1:
                    temp = cnts
                    secondMax = cnts[0]
                    for i in range(0, len(temp) - 1):
                        if cv2.contourArea(temp[i]) > cv2.contourArea(secondMax):
                            if cv2.contourArea(temp[i]) != cv2.contourArea(Max):
                                secondMax = temp[i]

                    # print(Max)
                    # print("Max")
                    rect2 = cv2.minAreaRect(secondMax)  # draw the min area rectangle
                    box2 = cv2.boxPoints(rect2)  # save the corner point to box
                    print('box2 = ', box2)

                    if rect2[1][0] != 0:
                        if rect2[1][1] < rect2[1][0]:
                            inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect2[1][0])
                            # print('rect2[1][0] = ', rect2[1][0])
                            cv2.putText(frame, "%.2fcm" % (inches * 2.54),
                                        (frame.shape[1] - 400, frame.shape[0] - 40),
                                        cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
                        else:
                            inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect2[1][1])
                            # print('rect2[1][0] = ', rect2[1][0])
                            cv2.putText(frame, "%.2fcm" % (inches * 2.54),
                                        (frame.shape[1] - 400, frame.shape[0] - 40),
                                        cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
                    cv2.drawContours(frame, [np.int0(box2)], -1, (255, 255, 255), 2)
                    # counter = counter + 1

                    # if counter == 2:
                    #     break
                # print(box)  # draw the rectangle

            cv2.imshow('camera', frame)  # show the frame
            cv2.waitKey(1)  # let the frame wait
        else:
            print("No picture")
    else:
        print("No access to camera")

cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()

# img = np.zeros((256,256,3),np.uint8)
#
# plt.imshow(img[:,:, ::-1])
#
# print(img[100,100])
# print(img[100,100,0])
# img[100,100]=(0,0,255)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    pass
    # url = 'https://cn.bing.com/'
    # strhtml = requests.get(url)
    # soup = BeautifulSoup(strhtml.text, 'lxml')
    # data = soup.select('#main>div>div.mtop.firstMod.clearfix>div.centerBox>ul.newsList>li>a')
    # print(strhtml.text)
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
