import os
import numpy as np
import cv2
import serial.tools.list_ports
import time
from pynput import keyboard

ball_color = 'red'  # choose color to recognize
color_dist = {'red': {'Lower': np.array([160, 100, 150]), 'Upper': np.array([200, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([48, 63, 84]), 'Upper': np.array([77, 255, 255])},
              }  # define the exact bound for each color
# 35 77 43 255 46 255
# 'green': {'Lower': np.array([35, 43, 117]), 'Upper': np.array([77, 255, 255])},
no_block_counter = 300
video_width = 1920
video_height = 1080
centre = 0
KNOWN_DISTANCE = 50
KNOWN_WIDTH = 1.64
global last_command_char
last_command_char = "1"
ACCData = [0.0] * 8
GYROData = [0.0] * 8
AngleData = [0.0] * 8
FrameState = 0  # 通过0x后面的值判断属于哪一种情况
Bytenum = 0  # 读取到这一段的第几位
CheckSum = 0  # 求和校验位

a = [0.0] * 3
w = [0.0] * 3
Angle = [0.0] * 3


def DueData(inputdata):  # 新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
    global FrameState  # 在局部修改全局变量，要进行global的定义
    global Bytenum
    global CheckSum
    global a
    global w
    global Angle
    for data in inputdata:  # 在输入的数据进行遍历
        # data = ord(data)
        if FrameState == 0:  # 当未确定状态的时候，进入以下判断
            if data == 0x55 and Bytenum == 0:  # 0x55位于第一位时候，开始读取数据，增大bytenum
                CheckSum = data
                Bytenum = 1
                continue
            elif data == 0x51 and Bytenum == 1:  # 在byte不为0 且 识别到 0x51 的时候，改变frame
                CheckSum += data
                FrameState = 1
                Bytenum = 2
            elif data == 0x52 and Bytenum == 1:  # 同理
                CheckSum += data
                FrameState = 2
                Bytenum = 2
            elif data == 0x53 and Bytenum == 1:
                CheckSum += data
                FrameState = 3
                Bytenum = 2
        elif FrameState == 1:  # acc    #已确定数据代表加速度

            if Bytenum < 10:  # 读取8个数据
                ACCData[Bytenum - 2] = data  # 从0开始
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):  # 假如校验位正确
                    a = get_acc(ACCData)
                CheckSum = 0  # 各数据归零，进行新的循环判断
                Bytenum = 0
                FrameState = 0
        elif FrameState == 2:  # gyro

            if Bytenum < 10:
                GYROData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    w = get_gyro(GYROData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
        elif FrameState == 3:  # angle

            if Bytenum < 10:
                AngleData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    Angle = get_angle(AngleData)
                    d = a + w + Angle
                    print("a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f Angle(deg):%10.3f %10.3f %10.3f" % d)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
    return Angle[2]


def DueData2(inputdata):  # 新增的核心程序，对读取的数据进行划分，各自读到对应的数组里
    global FrameState  # 在局部修改全局变量，要进行global的定义
    global Bytenum
    global CheckSum
    global a
    global w
    global Angle
    for data in inputdata:  # 在输入的数据进行遍历
        # data = ord(data)
        if FrameState == 0:  # 当未确定状态的时候，进入以下判断
            if data == 0x55 and Bytenum == 0:  # 0x55位于第一位时候，开始读取数据，增大bytenum
                CheckSum = data
                Bytenum = 1
                continue
            elif data == 0x51 and Bytenum == 1:  # 在byte不为0 且 识别到 0x51 的时候，改变frame
                CheckSum += data
                FrameState = 1
                Bytenum = 2
            elif data == 0x52 and Bytenum == 1:  # 同理
                CheckSum += data
                FrameState = 2
                Bytenum = 2
            elif data == 0x53 and Bytenum == 1:
                CheckSum += data
                FrameState = 3
                Bytenum = 2
        elif FrameState == 1:  # acc    #已确定数据代表加速度

            if Bytenum < 10:  # 读取8个数据
                ACCData[Bytenum - 2] = data  # 从0开始
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):  # 假如校验位正确
                    a = get_acc(ACCData)
                CheckSum = 0  # 各数据归零，进行新的循环判断
                Bytenum = 0
                FrameState = 0
        elif FrameState == 2:  # gyro

            if Bytenum < 10:
                GYROData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    w = get_gyro(GYROData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
        elif FrameState == 3:  # angle

            if Bytenum < 10:
                AngleData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    Angle = get_angle(AngleData)
                    d = a + w + Angle
                    print("a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f Angle(deg):%10.3f %10.3f %10.3f" % d)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0
    return Angle[1]


def get_acc(datahex):
    axl = datahex[0]
    axh = datahex[1]
    ayl = datahex[2]
    ayh = datahex[3]
    azl = datahex[4]
    azh = datahex[5]

    k_acc = 16.0

    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z -= 2 * k_acc

    return acc_x, acc_y, acc_z


def get_gyro(datahex):
    wxl = datahex[0]
    wxh = datahex[1]
    wyl = datahex[2]
    wyh = datahex[3]
    wzl = datahex[4]
    wzh = datahex[5]
    k_gyro = 2000.0

    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >= k_gyro:
        gyro_z -= 2 * k_gyro
    return gyro_x, gyro_y, gyro_z


def get_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180.0

    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >= k_angle:
        angle_z -= 2 * k_angle

    return angle_x, angle_y, angle_z


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
    global centre
    box = np.int0(box)
    most_right = 0
    most_left = video_width

    for k in range(0, 3):
        if box[k][0] >= most_right:
            most_right = box[k][0]
        if box[k][0] <= most_left:
            most_left = box[k][0]
    if centre == 0:
        if most_right < video_width / 2:
            return 0
        elif most_left > video_width / 2:
            return 2
        return 1
    elif centre == 1:
        if most_right < video_width / 5:
            return 0
        else:
            return 2
    elif centre == 2:
        if most_left > 4 * video_width / 5:
            return 2
        else:
            return 0


def is_in_outerlefthalf(box):  # determine if the color block is in the left half
    global centre
    box = np.int0(box)
    most_right = 0
    most_left = video_width

    for k in range(0, 3):
        if box[k][0] >= most_right:
            most_right = box[k][0]
        if box[k][0] <= most_left:
            most_left = box[k][0]

    if centre == 0:
        if most_right < video_width / 3:
            return 0
        elif most_left > 2 * video_width / 3:
            return 2
        return 1
    # elif centre == 1:
    #     if most_right < video_width / 5:
    #         return 0
    #     else:
    #         return 2
    # elif centre == 2:
    #     if most_left > 4 * video_width / 5:
    #         return 2
    #     else:
    #         return 0


def is_highhalf(box):  # determine if the color block is in the higher half
    box = np.int0(box)
    most_low = video_height
    most_high = 0

    for k in range(0, 3):
        if box[k][1] >= most_high:
            most_low = box[k][1]
        if box[k][1] <= most_low:
            most_high = box[k][1]

    if most_low < video_height / 2:  # 480/2 most_low for the lowest point
        return 0
    elif most_high > video_height / 2:
        return 2
    return 1


def is_parallel(dist1, dist2):  # determine if the two color blocks are parallel
    if abs(dist1 - dist2) < 80:
        return True
    else:
        return False


# function to get the distance between the middle point of color blocks and the center of the picture
def calculate_error(distance_1, distance_2, box_1, box_2):
    x1 = (box_1[0][0] + box_1[1][0] + box_1[2][0] + box_1[3][0]) / 4
    x2 = (box_2[0][0] + box_2[1][0] + box_2[2][0] + box_2[3][0]) / 4
    x_mean = (x1 + x2) / 2
    if centre == 0:
        if distance_1 - distance_2 > 30:
            if is_lefthalf(box_1) == 0:
                return x_mean - 2 * video_width / 3
            elif is_lefthalf(box_1) == 2:
                return x_mean - video_width / 3
            elif is_lefthalf(box_1) == 1:
                return 0
        elif distance_2 - distance_1 > 30:
            if is_lefthalf(box_2) == 0:
                return x_mean - 2 * video_width / 3
            elif is_lefthalf(box_2) == 2:
                return x_mean - video_width / 3
            elif is_lefthalf(box_2) == 1:
                return 0
        else:
            return x_mean - video_width / 2
    else:
        if centre == 1:
            if x2 > x1:
                return x2 - video_width / 3
            else:
                return x1 - video_width / 3
        elif centre == 2:
            if x2 < x1:  # x2 < x1
                return x2 - 2 * video_width / 3
            else:
                return x1 - 2 * video_width / 3


# function to determine if the two color blocks are on the edge
def on_edge(box_1, box_2):
    left_most = min(box_1[0][0], box_1[1][0], box_1[2][0], box_1[3][0], box_2[0][0], box_2[1][0], box_2[2][0],
                    box_2[3][0])
    right_most = max(box_1[0][0], box_1[1][0], box_1[2][0], box_1[3][0], box_2[0][0], box_2[1][0], box_2[2][0],
                     box_2[3][0])
    if left_most < 150 and right_most > video_width - 150:
        return True
    else:
        return False


def on_edge1(box_1):
    left_most = min(box_1[0][0], box_1[1][0], box_1[2][0], box_1[3][0])
    # right_most = max(box_1[0][0], box_1[1][0], box_1[2][0], box_1[3][0])
    if left_most > video_width - 300:
        return True
    else:
        return False


def on_edge2(box_2):
    # left_most = min(box_2[0][0], box_1[1][0], box_1[2][0], box_1[3][0])
    right_most = max(box_2[0][0], box_2[1][0], box_2[2][0], box_2[3][0])
    if right_most < 300:
        return True
    else:
        return False


# function to calculate  using image saved
def calculate_focal_distance(image_path):
    first_image = cv2.imread(image_path)

    marker = find_marker(first_image)

    focal_length = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

    print('focalLength = ', focal_length)

    return focal_length


img_path = "Picture3.png"  # image saved for focalDistance calculation
focalLength = calculate_focal_distance(img_path)  # calculate focal distance
plist = list(serial.tools.list_ports.comports())  # find serial
# print("serial names :", plist[0], plist[1])
last_command = 0  # declare the variable to store the time of the last command
# set up serials and COM
if len(plist) <= 0:
    print("no serial")
else:
    plist_0 = list(plist[0])
    # plist_1 = list(plist[1])
    print(plist_0)
    serialName = plist_0[0]  # plist_0[0] choose according to arduino ide
    serialFd = serial.Serial("COM11", 115200, timeout=0.1)  # define the serial
    # serial_imu = serial.Serial("COM7", 115200, timeout=0.5)
    print("serial name ", serialFd.name)

# time.sleep(2)
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


def adjust_passing_gate():  # function for passing gates
    global last_command_char
    if last_command_char != "w":
        go_ahead()
        time.sleep(15)
        last_command_char = "w"

    print("adjust & passing")


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


# def open_loop_adjusting(flag):
#     go_ahead()
#     time.sleep(5)
#
#     serial_imu.flushInput()
#     data_hex_function = serial_imu.read(33)
#     angle_z_function = DueData(data_hex_function)
#     if flag:
#         turn_right()
#         while 90 + angle_z_function > 0.2:
#             serial_imu.flushInput()
#             data_hex_function = serial_imu.read(33)
#             angle_z_function = DueData(data_hex_function)
#         print("turning right done")
#         serial_imu.write(bytearray([0xFF, 0xAA, 0x67]))
#         serial_imu.write(bytearray([0xFF, 0xAA, 0x52]))
#         # time.sleep(1)
#         go_ahead()
#         time.sleep(1)
#
#         serial_imu.flushInput()
#         data_hex_function = serial_imu.read(33)
#         angle_z_function = DueData(data_hex_function)
#         turn_left()
#         while 90 - angle_z_function > 0.2:
#             serial_imu.flushInput()
#             data_hex_function = serial_imu.read(33)
#             angle_z_function = DueData(data_hex_function)
#         print("turning left done")
#         serial_imu.write(bytearray([0xFF, 0xAA, 0x67]))
#         serial_imu.write(bytearray([0xFF, 0xAA, 0x52]))
#         go_ahead()
#         time.sleep(1)
#         global last_command_char
#         last_command_char = "w"
#     else:
#         turn_left()
#         while 90 - angle_z_function > 0.2:
#             serial_imu.flushInput()
#             data_hex_function = serial_imu.read(33)
#             angle_z_function = DueData(data_hex_function)
#
#         serial_imu.write(bytearray([0xFF, 0xAA, 0x67]))
#         serial_imu.write(bytearray([0xFF, 0xAA, 0x52]))
#         print("turning left done")
#         go_ahead()
#         time.sleep(1)
#
#         serial_imu.flushInput()
#         data_hex_function = serial_imu.read(33)
#         angle_z_function = DueData(data_hex_function)
#         turn_right()
#         while 90 + angle_z_function > 0.2:
#             serial_imu.flushInput()
#             data_hex_function = serial_imu.read(33)
#             angle_z_function = DueData(data_hex_function)
#
#         serial_imu.write(bytearray([0xFF, 0xAA, 0x67]))
#         serial_imu.write(bytearray([0xFF, 0xAA, 0x52]))
#         print("turning right done")
#         # time.sleep(1)
#         # global last_command_char
#         go_ahead()
#         time.sleep(1)
#         last_command_char = "w"


def pure_cv_open_loop_adjusting(flag1):
    global last_command_char
    go_ahead()
    time.sleep(5)
    if not flag1:
        turn_left()
        time.sleep(52)
        go_ahead()
        time.sleep(10)
        turn_right()
        time.sleep(58)
        go_ahead()
        last_command_char = "w"
    else:
        turn_right()
        time.sleep(58)
        go_ahead()
        time.sleep(10)
        turn_left()
        time.sleep(52)
        go_ahead()
        last_command_char = "w"


# function to send the next command
def next_command(dist_1, dist_2, box_1, box_2):
    global centre
    # if not on_edge(box_1, box_2):  # if the two color blocks are on the edges
    #     if is_parallel(dist_1, dist_2):  # if the two color blocks are from the same gate
    error = calculate_error(dist_1, dist_2, box_1, box_2)  # calculate the error
    if is_lefthalf(box_1) != is_lefthalf(box_2):
        if error > 10:
            print("right more")
            return "d"
            # turn_right()
        elif error < -10:
            print("left more")
            # turn_left()
            return "a"
        else:
            print("neither too left nor too right")
            return "w"

            # go_ahead()
    elif is_lefthalf(box_1) == 0 and is_lefthalf(box_2) == 0 and centre == 0:
        if is_in_outerlefthalf(box_1) == 0 and is_in_outerlefthalf(box_2) == 0:
            centre = 2
            print("centre = ", centre)
        print("both in left half")

        return "a"
        # turn_left()
    elif is_lefthalf(box_1) == 2 and is_lefthalf(box_2) == 2 and centre == 0:
        if is_in_outerlefthalf(box_1) == 2 and is_in_outerlefthalf(box_2) == 2:
            centre = 1
            print("centre = ", centre)
        print("both in right half")
        #
        return "d"


def compare_boxes(box1, box2):
    box1 = np.int0(box1)
    box2 = np.int0(box2)
    if box1[0][0] == box2[0][0] and box1[0][1] == box2[0][1] and box1[1][0] == box2[1][0] and box1[1][1] == box2[1][
        1] and box1[2][0] == box2[2][0] and box1[2][1] == box2[2][1] and box1[3][0] == box2[3][0] and box1[3][1] == \
            box2[3][1]:
        return False
    return True


def change_last_command_char(char):
    global last_command_char
    last_command_char = char


def find_center(box):
    return (box[0][0] + box[1][0] + box[2][0] + box[3][0]) / 4


keyboard_listener()
# code for going rapidly without camera

# using camera below
pump()
time.sleep(1)
# go_ahead()
# time.sleep(120)
if ball_color == 'green':
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('camera', cv2.WINDOW_NORMAL)  # open a window to show
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
# start video capture
#  switching to camera control
middle()
last_command_char = "g"
time.sleep(5)
print("middle finished")
if ball_color == 'green':
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
                        if cv2.contourArea(Max) < 200:  # if the first block is too small ignore it
                            # print("box1 too small ignored")
                            distance_1 = 0
                    print("distance1 =", distance_1)
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
                        if rect_2[1][0] != 0 and compare_boxes(box1, box2):
                            if rect_2[1][1] < rect_2[1][0]:
                                inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect_2[1][0])
                                distance_2 = inches * 2.4
                            else:
                                inches = distance_to_camera(KNOWN_WIDTH, focalLength, rect_2[1][1])
                                distance_2 = inches * 2.4
                            if cv2.contourArea(secondMax) < 200:  # if the first block is too small ignore it
                                # print("box2 too small ignored")
                                distance_2 = 0
                        cv2.drawContours(frame, [np.int0(box2)], -1, (255, 255, 255), 2)
                        print("distance2 =", distance_2)
                        # print("box2 ==", box2)
                cv2.imshow('camera', frame)  # show the frame
                cv2.waitKey(1)  # let the frame wait

                # decision part
                if distance_1 != 0 and distance_2 != 0:
                    if centre == 0:
                        if is_parallel(distance_1, distance_2):
                            print("parallel")
                            if (on_edge2(box1) and on_edge1(box2)) or (on_edge2(box2) and on_edge1(box1)):
                                adjust_passing_gate()
                                print("both on edge")
                            elif on_edge1(box1) or on_edge2(box1):
                                print("box1 on edge")
                                if on_edge1(box1):
                                    if last_command_char != "d":
                                        turn_right()
                                        last_command_char = "d"
                                else:
                                    if last_command_char != "a":
                                        turn_left()
                                        last_command_char = "a"
                            elif on_edge1(box2) or on_edge2(box2):
                                print("box2 on edge")
                                if on_edge1(box2):
                                    if last_command_char != "d":
                                        turn_right()
                                        last_command_char = "d"
                                else:
                                    if last_command_char != "a":
                                        turn_left()
                                        last_command_char = "a"
                            else:
                                bug_king = next_command(distance_1, distance_2, box1, box2)
                                if last_command_char != bug_king:
                                    if bug_king == "w":
                                        go_ahead()
                                        # # print("print w")
                                        last_command_char = "w"

                                    elif bug_king == "a":
                                        turn_left()

                                        last_command_char = "a"

                                    elif bug_king == "d":
                                        turn_right()

                                        last_command_char = "d"

                                print("Using next command")
                        else:
                            print("not parallel")

                            if is_lefthalf(box1) == 0 and centre == 0:
                                if distance_1 > 160:
                                    if last_command_char != "a":
                                        turn_left()
                                        last_command_char = "a"
                                else:
                                    if last_command_char != "d":
                                        turn_right()
                                        last_command_char = "d"
                                    print("in left half")

                            elif is_lefthalf(box1) == 2 and centre == 0:
                                if distance_1 > 160:
                                    if last_command_char != "d":
                                        turn_right()
                                        last_command_char = "d"
                                else:
                                    if last_command_char != "a":
                                        turn_left()
                                        last_command_char = "a"
                                print("in right half")
                    else:
                        if (on_edge2(box1) and on_edge1(box2)) or (on_edge2(box2) and on_edge1(box1)):
                            adjust_passing_gate()
                            print("both on edge")
                            centre = 0
                            print("centre = 0")
                        bug_king = next_command(distance_1, distance_2, box1, box2)
                        if last_command_char != bug_king:

                            if bug_king == "w":
                                go_ahead()
                                # # print("print w")
                                last_command_char = "w"

                            elif bug_king == "a":
                                turn_left()

                                last_command_char = "a"
                            elif bug_king == "d":
                                turn_right()
                                last_command_char = "d"
                elif distance_1 == 0 and distance_2 == 0:
                    # global open_loop_adjusting_counter
                    # open_loop_adjusting_counter = 0
                    no_block_counter = no_block_counter + 1

                    if centre == 0:
                        if last_command_char != "w":
                            go_ahead()
                            last_command_char = "w"
                    elif centre == 1:
                        if last_command_char != "a":
                            turn_left()
                            last_command_char = "a"
                    elif centre == 2:
                        if last_command_char != "d":
                            turn_right()
                            last_command_char = "d"

                    if no_block_counter > 300:
                        ball_color = 'red'
                        break
                else:
                    if distance_1 != 0:
                        print("only one color block")
                        # turn_left()
                        # global open_loop_adjusting_counter
                        if on_edge1(box1):
                            print("on edge")
                            go_ahead()
                            time.sleep(10)
                            if last_command_char != "w":
                                last_command_char = "w"
                            # print("last command char changed = ", last_command_char)
                        else:
                            print("not on edge")
                            # open_loop_adjusting_counter = open_loop_adjusting_counter + 1
                            # if open_loop_adjusting_counter == 150:
                            if is_lefthalf(box1) == 0:
                                print("left")

                                open_loop_adjusting_counter = 0
                                if distance_1 > 160:
                                    if last_command_char != "a":
                                        turn_left()
                                        last_command_char = "a"
                                else:
                                    if last_command_char != "d":
                                        turn_right()
                                        last_command_char = "d"

                            elif is_lefthalf(box1) == 2:
                                print("right")
                                # open_loop_adjusting(False)
                                # pure_cv_open_loop_adjusting(False)
                                open_loop_adjusting_counter = 0
                                if distance_1 > 160:
                                    if last_command_char != "d":
                                        turn_right()
                                        last_command_char = "d"
                                else:
                                    if last_command_char != "a":
                                        turn_left()
                                        last_command_char = "a"
                                    # centre = 0
                                    # print("centre = 0")
                            elif is_lefthalf(box1) == 1:
                                if distance_1 > 160:
                                    if last_command_char != "w":
                                        go_ahead()
                                        last_command_char = "w"
                                else:
                                    if centre == 1:
                                        if last_command_char != "a":
                                            turn_left()
                                            last_command_char = "a"
                                    elif centre == 2:
                                        if last_command_char != "d":
                                            turn_right()
                                            last_command_char = "d"
                                    elif centre == 0:
                                        print("G")
            else:
                print("No picture")
        else:
            print("No access to camera")

    cap.release()
    # cv2.waitKey(0)
    cv2.destroyAllWindows()
    # time.sleep(1)

os.system('test.py')
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # start video capture
cv2.namedWindow('camera2', cv2.WINDOW_NORMAL)  # open a window to show
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
# print(cap.isOpened(), serialFd.isOpen())
while cap.isOpened() and serialFd.isOpen():  # while the capture is open
    ret, frame = cap.read()  # read ret and frame
    if ret:
        if frame is not None:  # have image
            # print("1")
            gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)  # using GaussianBlur
            hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # From BGR to HSV
            erode_hsv = cv2.erode(hsv, None, iterations=2)  # erode to reduce noise
            in_range_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
            # delete backgrounds
            cnt_s = cv2.findContours(in_range_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            # print("2")
            # serial_imu.flushInput()
            # data_hex = serial_imu.read(33)
            # angle_z = DueData(data_hex)
            # print(DueData(data_hex))
            # print("3")
            # initialize the variables
            distance_1 = 0
            distance_2 = 0
            box1 = 0
            box2 = 0
            cent = [1, 0, 0, 0, 0]
            error = [960, 960, 960, 960, 960]
            counter = 0
            for i in range(0, len(cnt_s)):
                if cv2.contourArea(cnt_s[i]) > 200 and counter < 5:
                    rect = cv2.minAreaRect(cnt_s[i])
                    box = cv2.boxPoints(rect)
                    cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 2)
                    cent[counter] = find_center(box)
                    error[counter] = video_width / 2 - cent[counter]
                    counter = counter + 1
            print(cent)
            print(error)
            for i in range(0, len(cent)-1):
                for j in range(i+1, len(cent)):
                    if abs(cent[i]-cent[j]) < 150:
                        if abs(error[i]) < abs(error[j]):
                            cent[j] = 0
                            error[j] = 960
                        else:
                            cent[i] = 0
                            error[i] = 960
            print(cent)
            print(error)
            min_error = 960
            min_pointer = 0
            for i in range(0, len(cent)):
                if abs(error[i]) < min_error:
                    min_error = abs(error[i])
                    min_pointer = i
            sec_error = 960
            sec_pointer = 0
            for i in range(0, len(cent)):
                if sec_error > abs(error[i]) > min_error:
                    sec_error = abs(error[i])
                    sec_pointer = i
            print(min_pointer, sec_pointer)
            middle_point = (cent[min_pointer] + cent[sec_pointer]) / 2
            middle_error = video_width / 2 - middle_point
            if middle_error < -8:
                if last_command_char != "d":
                    turn_right()
                    last_command_char = "d"
            elif middle_error > 8:
                if last_command_char != "a":
                    turn_left()
                    last_command_char = "a"
            else:
                if last_command_char != "w":
                    go_ahead()
                    last_command_char = "w"
                time.sleep(60)
            cv2.imshow('camera', frame)  # show the frame
            cv2.waitKey(1)  # let the frame wait
        else:
            print("No picture")
    else:
        print("No access to camera")

cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()
