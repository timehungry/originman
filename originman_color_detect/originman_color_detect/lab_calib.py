#!/usr/bin/env python3
import cv2
import numpy as np

print("=== 机器人 LAB 颜色校准（SSH 专用）===")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 红色初始值
L_MIN = 0
A_MIN = 150
B_MIN = 142


L_MAX = 255
A_MAX = 255
B_MAX = 255

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头失败")
        break

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lower = np.array([L_MIN, A_MIN, B_MIN])
    upper = np.array([L_MAX, A_MAX, B_MAX])
    mask = cv2.inRange(lab, lower, upper)

    cv2.imwrite("/tmp/test.jpg", frame)
    cv2.imwrite("/tmp/mask.jpg", mask)

    print("当前 LAB 值：")
    print(f"min: [{L_MIN}, {A_MIN}, {B_MIN}]")
    print(f"max: [{L_MAX}, {A_MAX}, {B_MAX}]")
    print("已保存：/tmp/test.jpg 和 /tmp/mask.jpg")
    break

cap.release()