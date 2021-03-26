# tello Face Tracking, HaarCascade, PID Control

import cv2
import numpy as np
import time
from djitellopy import tello

me = tello.Tello()
me.connect()
print(me.get_battery())

me.streamon()
me.takeoff()

##send_rc_control(self, left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
# me.send_rc_control(0, 0, 15, 0)
# time.sleep(2.2)

w, h = 360, 240
fbRange = [6200, 6800]

pid = [0.4, 0.4, 0]
pError = 0

# 비디오 처리
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('output.avi', fourcc, 25.0, (w, h))


def findFace(img):
    faceCascade = cv2.CascadeClassifier("./haarcascade_frontalface_default.xml")
    imgGrey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGrey, 1.2, 8)

    myFaceListC = []
    myFaceListArea = []

    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        cv2.circle(img, (cx, cy), 5, (0, 255, 0))
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)

    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]


def trackFace(info, w, pid, pError):
    ###### yaw PID control
    # pid : [0.4, 0.4, 0]:
    area = info[1]  # area
    x, y = info[0]  # x,y
    fb = 0  # [6200, 6800]

    error = x - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))   # yaw

    if x == 0:
        speed = 0
        error = 0

    ##### forward/ backward control
    if area > fbRange[0] and area < fbRange[1]:
        fb = 0

    if area > fbRange[1]:
        fb = -20

    elif area < fbRange[0] and area != 0:
        fb = 20

    # print(error, fb)
    print('speed=', speed, 'fb=', fb)

    # if x == 0:
    #     speed = 0
    #     error = 0

    me.send_rc_control(0, fb, 0, speed)
    return error


# cap = cv2.VideoCapture(0)
while True:
    # _, img = cap.read()
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w, h))

    img, info = findFace(img)

    pError = trackFace(info, w, pid, pError)
    # print("Center", info[0], "Area", info[1])

    cv2.imshow("Output", img)
    # 비디오 저장
    out.write(img)

    key = cv2.waitKey(1) & 0xff
    if key == 27:  # ESC
        break
    elif key == ord('t'):
        me.takeoff()
        me.send_rc_control(0, 0, 20, 0)
        # time.sleep(1.0)
    elif key == ord('f'):
        me.move_forward(10)
    elif key == ord('b'):
        me.move_back(10)
    elif key == ord('l'):
        me.move_left(10)
    elif key == ord('r'):
        me.move_right(10)
    elif key == ord('c'):
        me.rotate_clockwise(10)
    elif key == ord('v'):
        me.rotate_counter_clockwise(10)
    elif key == ord('u'):
        me.move_up(10)
    elif key == ord('d'):
        me.move_down(10)

cv2.destroyAllWindows()
me.land()

