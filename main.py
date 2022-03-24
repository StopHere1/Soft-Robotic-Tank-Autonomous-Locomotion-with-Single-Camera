# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import numpy as np
import cv2
import serial.tools.list_ports
import time
import _thread
from pynput import keyboard

# print(cv.__version__)
# img = cv.imread("E:\SUSTECH\Student Work\FORSTUDY\Lab\IEEE SoftRobot Competition\ComputerVersion\GreenBlocks.jpg",1)
# cv.imshow("1",img)
# cv.waitKey()

ball_color = 'red'  # choose color to recognize
color_dist = {'red': {'Lower': np.array([160, 120, 150]), 'Upper': np.array([200, 200, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 46]), 'Upper': np.array([77, 255, 255])},
              }  # define the exact bound for each color

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # start video capture
cv2.namedWindow('camera', cv2.WINDOW_AUTOSIZE)  # open a window to show
# print("1")

video_width = 1920
video_height = 1080

KNOWN_DISTANCE = 50
KNOWN_WIDTH = 1.64
global last_command_char
last_command_char = "1"


def find_marker(image):  # function to find color block
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
    edged_img = cv2.Canny(gray_img, 35, 125)

    (cnt, _) = cv2.findContours(edged_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    c = max(cnt, key=cv2.contourArea)

    return cv2.minAreaRect(c)


def distance_to_camera(known_width, focal_length, per_width):  # function to get the distance to the camera(inch)
    # if perWidth != 0:
    return (known_width * focal_length) / per_width


# return 0


def is_lefthalf(box):  # determine if the color block is in the left half
    box = np.int0(box)
    most_right = 0
    most_left = video_width

    for k in range(0, 3):
        if box[k][0] >= most_right:
            most_right = box[k][0]
        if box[k][0] <= most_left:
            most_left = box[k][0]

    if most_right < video_width/2:  # 640/2
        return 0
    elif most_left > video_width/2:
        return 2
    return 1


def is_highhalf(box):  # determine if the color block is in the higher half
    box = np.int0(box)
    most_low = video_height
    most_high = 0

    for k in range(0, 3):
        if box[k][1] >= most_high:
            most_low = box[k][1]
        if box[k][1] <= most_low:
            most_high = box[k][1]

    if most_low < video_height/2:  # 480/2 most_low for the lowest point
        return 0
    elif most_high > video_height/2:
        return 2
    return 1


def is_parallel(dist1, dist2):  # determine if the two color blocks are parallel
    if abs(dist1 - dist2) < 100:
        return True
    else:
        return False


# function to get the distance between the middle point of color blocks and the center of the picture
def calculate_error(box_1, box_2):
    x1 = (box_1[0][0] + box_1[1][0] + box_1[2][0] + box_1[3][0]) / 4
    x2 = (box_2[0][0] + box_2[1][0] + box_2[2][0] + box_2[3][0]) / 4
    x_mean = (x1 + x2) / 2
    return x_mean - video_width/2


# function to determine if the two color blocks are on the edge
def on_edge(box_1, box_2):
    left_most = min(box_1[0][0], box_1[1][0], box_1[2][0], box_1[3][0], box_2[0][0], box_2[1][0], box_2[2][0],
                    box_2[3][0])
    right_most = max(box_1[0][0], box_1[1][0], box_1[2][0], box_1[3][0], box_2[0][0], box_2[1][0], box_2[2][0],
                     box_2[3][0])
    if left_most < 50 and right_most > video_width-50:
        return True
    else:
        return False


# function to calculate focal_distance using image saved
def calculate_focal_distance(image_path):
    first_image = cv2.imread(image_path)

    marker = find_marker(first_image)

    focal_length = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

    print('focalLength = ', focal_length)

    return focal_length


img_path = "Picture3.png"  # image saved for focalDistance calculation
focalLength = calculate_focal_distance(img_path)  # calculate focal distance
plist = list(serial.tools.list_ports.comports())  # find serial
last_command = 0  # declare the variable to store the time of the last command
# set up serials and COM
if len(plist) <= 0:
    print("no serial")
else:
    plist_0 = list(plist[0])
    print(plist_0)
    serialName = plist_0[0]  # plist_0[0] choose according to arduino ide
    serialFd = serial.Serial(serialName, 115200, timeout=0.001)  # define the serial
    print("serial name ", serialFd.name)

time.sleep(2)
# Code for sending commands starts here


def go_ahead():  # 3.5cm per cycle when last_command = 2 s
    serialFd.write("w".encode())
    global last_command
    last_command = 2
    print("Forward")


def turn_left():
    serialFd.write("a".encode())
    global last_command
    last_command = 1
    print("turn left")


def turn_right():  # 7 degree per cycle when last_command = 2 s
    serialFd.write("d".encode())
    global last_command
    last_command = 1
    print("turn right")


def middle():
    serialFd.write("g".encode())
    global last_command
    last_command = 4
    print("chou")


def middle2():  # inverse function for middle()
    serialFd.write("h".encode())
    global last_command
    last_command = 4.5
    print("fang")


def pump():
    serialFd.write("q".encode())
    global last_command
    last_command = 0
    print("q")


def pump2():  # inverse function for pump()
    serialFd.write("e".encode())
    global last_command
    last_command = 0
    print("e")


def passing_gate():  # function for passing gates
    for times in range(1, 10):
        serialFd.write("w".encode())
    global last_command
    last_command = 20
    print("passing")


# Code for sending commands end here


#  Keyboard listening code begins here
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
    elif key.char == 'a':
        turn_left()
    elif key.char == 'd':
        turn_right()
    elif key.char == 'g':
        middle()
    elif key.char == 'h':
        middle2()
    elif key.char == 'q':
        pump()
    elif key.char == 'e':
        pump2()
    elif key.char == 's':
        return False


# keyboard_listener()  # open keyboard listening
# pump()

#  Keyboard listening code ends here


# function to send the next command
def next_command(dist_1, dist_2, box_1, box_2):
    if not on_edge(box_1, box_2):  # if the two color blocks are on the edges
        if is_parallel(dist_1, dist_2):  # if the two color blocks are from the same gate
            error = calculate_error(box_1, box_2)  # calculate the error
            if is_lefthalf(box_1) != is_lefthalf(box_2):
                if error > 40:
                    print("right more")
                    return "d"
                    # turn_right()
                elif error < -40:
                    print("left more")
                    # turn_left()
                    return "a"
                else:
                    print("neither too left nor too right")
                    return "w"
                    # go_ahead()
            elif is_lefthalf(box_1) and is_lefthalf(box_2):
                print("both in left half")
                return "a"
                # turn_left()
            elif not is_lefthalf(box_1) and not is_lefthalf(box_2):
                print("both in right half")
                # turn_right()
                return "d"
        else:
            if is_lefthalf(box_1):
                return "a"
            else:
                return "d"
            print("not parallel")
            # turn_left()
    else:
        return "p"
        # passing_gate()


def compare_boxes(box1, box2):
    if box1[0][0] == box2[0][0] and box1[0][1] == box2[0][1] and box1[1][0] == box2[1][0] and box1[1][1] == box2[1][
        1] and box1[2][0] == box2[2][0] and box1[2][1] == box2[2][1] and box1[3][0] == box2[3][0] and box1[3][1] == \
            box2[3][1]:
        return False
    return True


def change_last_command_char(char):
    global last_command_char
    last_command_char = char
# print("1")
# pump()


cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


while cap.isOpened() and serialFd.isOpen():  # while the capture is open
    ret, frame = cap.read()  # read ret and frame
    if ret:
        if frame is not None:  # have image
            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # using GaussianBlur
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # From BGR to HSV
            erode_hsv = cv2.erode(hsv, None, iterations=2)  # erode to reduce noise
            in_range_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
            # delete backgrounds
            cnt_s = cv2.findContours(in_range_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

            # initialize the variables
            distance_1 = 0
            distance_2 = 0
            box1 = 0
            box2 = 0
            if len(cnt_s) != 0:
                Max = max(cnt_s, key=cv2.contourArea)  # find outer edges of the rectangle
                rect_1 = cv2.minAreaRect(Max)  # draw the min area rectangle
                box1 = cv2.boxPoints(rect_1)  # save the corner point to box
                if rect_1[1][0] != 0:
                    if rect_1[1][1] < rect_1[1][0]:
                        inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect_1[1][0])
                        distance_1 = inches * 2.4
                    else:
                        inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect_1[1][1])
                        distance_1 = inches * 2.4
                    if cv2.contourArea(Max) < 100:  # if the first block is too small ignore it
                        print("box1 too small ignored")
                        distance_1 = 0
                print("distance1 =", distance_1)
                cv2.drawContours(frame, [np.int0(box1)], -1, (0, 255, 255), 2)
                print("box1 ==", box1)
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
                    if rect_2[1][0] != 0 and compare_boxes(box1, box2):
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
                        if cv2.contourArea(secondMax) < 100:  # if the first block is too small ignore it
                            print("box2 too small ignored")
                            distance_2 = 0
                    cv2.drawContours(frame, [np.int0(box2)], -1, (255, 255, 255), 2)
                    print("distance2 =", distance_2)
                    print("box2 ==", box2)
            cv2.imshow('camera', frame)  # show the frame
            cv2.waitKey(1)  # let the frame wait
            # messages transmitting to arduino
            if distance_1 != 0 and distance_2 != 0:
                if last_command_char != next_command(distance_1, distance_2, box1, box2):
                    # global last_command_char
                    last_command_char = next_command(distance_1, distance_2, box1, box2)
                    print("last command char = ", last_command_char)

                    if last_command_char == 'w':
                        go_ahead()
                    elif last_command_char == 'a':
                        turn_left()
                    elif last_command_char == 'd':
                        turn_right()
                    elif last_command_char == 'g':
                        middle()
                    elif last_command_char == 'h':
                        middle2()
                    elif last_command_char == 'q':
                        pump()
                    elif last_command_char == 'e':
                        pump2()
                print("Using next command")
            elif distance_1 == 0 and distance_2 == 0:
                if last_command_char != "w":
                    # global last_command_char
                    last_command_char = "w"
                    go_ahead()
                # go_ahead()
                print("did not find color blocks")
            else:
                # turn_left()
                if last_command_char != "a":
                    # global last_command_char
                    last_command_char = "a"
                    turn_left()
                print("only one color block")
            # waiting till the work done
            # time.sleep(last_command)
            # print(last_command)
            #
            # while 1:
            #     message = serialFd.readline()
            #     if message.decode() == "1":
            #         break

        else:
            print("No picture")
    else:
        print("No access to camera")

cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
