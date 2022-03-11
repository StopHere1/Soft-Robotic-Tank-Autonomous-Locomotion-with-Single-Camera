# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np
import cv2
import serial.tools.list_ports
import time
from pynput import keyboard

# print(cv.__version__)
# img = cv.imread("E:\SUSTECH\Student Work\FORSTUDY\Lab\IEEE SoftRobot Competition\ComputerVersion\GreenBlocks.jpg",1)
# cv.imshow("1",img)
# cv.waitKey()

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


def find_marker(image):  # function to find color block
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
    edged_img = cv2.Canny(gray_img, 35, 125)

    (cnts, _) = cv2.findContours(edged_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnts, key=cv2.contourArea)

    return cv2.minAreaRect(c)


# IMAGE_PATH = ["Picture1.jpg", "Picture2.jpg"]

# focalLength = 2.8


# image = cv2.imread(IMAGE_PATH[0])
# marker = find_marker(image)
# focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH


def distance_to_camera(known_width, focal_length, per_width):  # function to get the distance to the camera(inch)
    # if perWidth != 0:
    return (known_width * focal_length) / per_width


# return 0


def calculate_focal_distance(image_path):  # function to calculate focal_distance using image saved
    first_image = cv2.imread(image_path)

    marker = find_marker(first_image)

    focal_length = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

    print('focalLength = ', focalLength)

    return focal_length


def is_lefthalf(box):  # determine if the color block is in the left half
    box = np.int0(box)
    most_right = 0
    most_left = 640

    for k in range(0, 3):
        if box[k][0] >= most_right:
            most_right = box[k][0]
        if box[k][0] <= most_left:
            most_left = box[k][0]

    if most_right < 320:  # 640/2
        return 0
    elif most_left > 320:
        return 2
    return 1


def is_highhalf(box):  # determine if the color block is in the higher half
    box = np.int0(box)
    most_low = 480
    most_high = 0

    for k in range(0, 3):
        if box[k][1] >= most_high:
            most_low = box[k][1]
        if box[k][1] <= most_low:
            most_high = box[k][1]

    if most_low < 240:  # 480/2
        return 0
    elif most_high > 240:
        return 2
    return 1


def is_parallel(dist1, dist2):  # determine if the color blocks are parallel
    if abs(dist1 - dist2) < 2:
        return True
    else:
        return False


# function to get the distance between the middle point of color blocks and the center of the picture
def calculate_error(box_1, box_2):
    x1 = (box_1[0][0] + box_1[1][0] + box_1[2][0] + box_1[3][0]) / 4
    x2 = (box_2[0][0] + box_2[1][0] + box_2[2][0] + box_2[3][0]) / 4
    x_mean = (x1 + x2) / 2
    return x_mean - 320


img_path = "Picture1.jpg"  # image saved for focalDistance calculation
focalLength = calculate_focal_distance(img_path)
plist = list(serial.tools.list_ports.comports())
last_command = 0
# set up serials and COM
if len(plist) <= 0:
    print("no serial")
else:
    plist_0 = list(plist[0])
    print(plist_0)
    serialName = 'COM5'  # plist_0[0] choose according to arduino ide
    serialFd = serial.Serial(serialName, 9600, timeout=.1)
    print("serial name ", serialFd.name)


#  Keyboard listening code begins here
def go_ahead():
    serialFd.write("w".encode())
    global last_command
    last_command = 1
    print("w")


def turn_left():
    serialFd.write("a".encode())
    global last_command
    last_command = 1
    print("a")


def turn_right():
    serialFd.write("d".encode())
    global last_command
    last_command = 1
    print("d")


def middle():
    serialFd.write("g".encode())
    global last_command
    last_command = 4
    print("g")


def middle2():
    serialFd.write("h".encode())
    global last_command
    last_command = 4.5
    print("h")


def pump():
    serialFd.write("q".encode())
    global last_command
    last_command = 0
    print("q")


def pump2():
    serialFd.write("e".encode())
    global last_command
    last_command = 0
    print("e")


def keyboard_listener():
    while True:
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()


def on_press(key):
    try:
        print(f'letter: {key.char}')
    except AttributeError:
        print(f'key= {key}')


def on_release(key):
    if key.char == 'w':
        go_ahead()
        return False
    elif key.char == 'a':
        turn_left()
        return False
    elif key.char == 'd':
        turn_right()
        return False
    elif key.char == 'g':
        middle()
    elif key.char == 'h':
        middle2()
    elif key.char == 'q':
        pump()
    elif key.char == 'e':
        pump2()
        return False


keyboard_listener()


#  Keyboard listening code ends here


def next_command(dist_1, dist_2, box_1, box_2):
    if is_parallel(dist_1, dist_2):
        error = calculate_error(box_1, box_2)
        if error > 2:
            turn_left()
        elif error < -2:
            turn_right()
        else:
            go_ahead()
    else:
        turn_left()


while cap.isOpened():  # while the capture is open
    ret, frame = cap.read()  # read ret and frame
    if ret:
        if frame is not None:  # have image
            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # using GaussianBlur
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # From BGR to HSV
            erode_hsv = cv2.erode(hsv, None, iterations=2)  # erode to reduce noise
            inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
            # delete backgrounds
            cnt_s = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            # find contours
            # print(frame.shape[0], ' ', frame.shape[1])  # 480 * 640 zero locates at
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
            distance_1 = 0
            distance_2 = 0
            box1 = 0
            box2 = 0
            if len(cnt_s) != 0:
                # counter = 0

                # cnts.sort(key=cv2.contourArea(cnts), reverse=False)

                Max = max(cnt_s, key=cv2.contourArea)  # find outer edges of the rectangle
                rect_1 = cv2.minAreaRect(Max)  # draw the min area rectangle
                box1 = cv2.boxPoints(rect_1)  # save the corner point to box
                if rect_1[1][0] != 0:
                    if rect_1[1][1] < rect_1[1][0]:
                        inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect_1[1][0])
                        distance_1 = inches * 2.4
                        # cv2.putText(frame, "%.2fcm" % (inches * 2.54), (frame.shape[1] - 200, frame.shape[0] - 20),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
                    else:
                        inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect_1[1][1])
                        distance_1 = inches * 2.4
                        # cv2.putText(frame, "%.2fcm" % (inches * 2.54), (frame.shape[1] - 200, frame.shape[0] - 20),
                        #             cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)

                # print('distance1 = ', distance_1)
                cv2.drawContours(frame, [np.int0(box1)], -1, (0, 255, 255), 2)
                if len(cnt_s) > 1:
                    temp = cnt_s
                    secondMax = cnt_s[0]
                    for i in range(0, len(temp) - 1):
                        if cv2.contourArea(temp[i]) > cv2.contourArea(secondMax):
                            if cv2.contourArea(temp[i]) != cv2.contourArea(Max):
                                secondMax = temp[i]

                    rect_2 = cv2.minAreaRect(secondMax)  # draw the min area rectangle
                    box2 = cv2.boxPoints(rect_2)  # save the corner point to box
                    # print('box2 = ', box2)
                    if rect_2[1][0] != 0:
                        if rect_2[1][1] < rect_2[1][0]:
                            inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect_2[1][0])
                            distance_2 = inches * 2.4
                            # print('rect2[1][0] = ', rect2[1][0])
                            # cv2.putText(frame, "%.2fcm" % (inches * 2.54),
                            #             (frame.shape[1] - 400, frame.shape[0] - 40),
                            #             cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
                        else:
                            inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect_2[1][1])
                            distance_2 = inches * 2.4
                            # print('rect2[1][0] = ', rect2[1][0])
                            # cv2.putText(frame, "%.2fcm" % (inches * 2.54),
                            #             (frame.shape[1] - 400, frame.shape[0] - 40),
                            #             cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)
                    # print('distance2 = ', distance_2)
                    cv2.drawContours(frame, [np.int0(box2)], -1, (255, 255, 255), 2)
                    # counter = counter + 1

                    # if counter == 2:
                    #     break
                # print(box)  # draw the rectangle

            cv2.imshow('camera', frame)  # show the frame
            cv2.waitKey(1)  # let the frame wait

            # messages transmitting to arduino
            if distance_1 != 0 and distance_2 != 0 and box1 != 0 and box2 != 0:
                next_command(distance_1, distance_2, box1, box2)

            # waiting till the work done
            time.sleep(last_command)
            # check response
            # while 1:
            #     if serialFd.readline() == 1:
            #         break
            #     continue
            # print("Command Done!")

        else:
            print("No picture")
    else:
        print("No access to camera")

cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
