import cv2
import numpy as np
ball_color = 'green'  # choose color to recognize
color_dist = {'red': {'Lower': np.array([160, 120, 150]), 'Upper': np.array([200, 200, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([30, 50, 160]), 'Upper': np.array([102, 130, 192])},
              }
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # start video capture
cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)  # open a window to show
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


def empty(a):
    pass


cv2.namedWindow("HSV")
cv2.resizeWindow("HSV", 640, 300)
cv2.createTrackbar("HUE Min", "HSV", 160, 179, empty)
cv2.createTrackbar("SAT Min", "HSV", 100, 255, empty)
cv2.createTrackbar("VALUE Min", "HSV", 150, 255, empty)
cv2.createTrackbar("HUE Max", "HSV", 200, 179, empty)
cv2.createTrackbar("SAT Max", "HSV", 200, 255, empty)
cv2.createTrackbar("VALUE Max", "HSV", 255, 255, empty)


def compare_area(area1, area2):
    rect1 = cv2.minAreaRect(area1)
    rect2 = cv2.minAreaRect(area2)
    bo1 = np.int0(cv2.boxPoints(rect1))
    bo2 = np.int0(cv2.boxPoints(rect2))
    x1 = (bo1[0][0] + bo1[1][0] + bo1[2][0] + bo1[3][0]) / 4
    x2 = (bo2[0][0] + bo2[1][0] + bo2[2][0] + bo2[3][0]) / 4
    error = x1 - x2
    print(50 > error > -50)
    return 50 > error > -50


while cap.isOpened():  # while the capture is open
    ret, frame = cap.read()  # read ret and frame
    h_min = cv2.getTrackbarPos("HUE Min", "HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    if ret:
        if frame is not None:
            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # using GaussianBlur
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # From BGR to HSV
            erode_hsv = cv2.erode(hsv, None, iterations=2)  # erode to reduce noise
            in_range_hsv = cv2.inRange(erode_hsv, lower, upper)  ## color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
            # delete backgrounds
            cnt_s = cv2.findContours(in_range_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            if len(cnt_s) != 0:
                Max = max(cnt_s, key=cv2.contourArea)  # find outer edges of the rectangle
                rect_1 = cv2.minAreaRect(Max)  # draw the min area rectangle
                box1 = cv2.boxPoints(rect_1)  # save the corner point to box
                # print("distance1 =", distance_1)
                cv2.drawContours(frame, [np.int0(box1)], -1, (0, 255, 255), 2)
                # print("box1 ==", box1)
                # find the second_largest color block
                if len(cnt_s) > 1:
                    temp = cnt_s
                    secondMax = cnt_s[0]
                    for i in range(0, len(temp) - 1):
                        if cv2.contourArea(temp[i]) > cv2.contourArea(secondMax):
                            if cv2.contourArea(temp[i]) != cv2.contourArea(Max):
                                secondMax = temp[i]

                    rect_2 = cv2.minAreaRect(secondMax)  # draw the min area rectangle
                    box2 = cv2.boxPoints(rect_2)  # save the corner point to box
                    cv2.drawContours(frame, [np.int0(box2)], -1, (255, 255, 255), 2)
                    # print("distance2 =", distance_2)
                    # print("box2 ==", box2)
            cnt_s = cv2.findContours(in_range_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            cv2.imshow('in_range', in_range_hsv)
            cv2.imshow('camera', frame)  # show the frame
            cv2.waitKey(1)  # let the frame wait
    else:
        print("No picture")
else:
    print("No access to camera")

cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
