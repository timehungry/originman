#!/usr/bin/env python3
# encoding:utf-8
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool  # 新增 Bool 用于接收语音信号
import numpy as np
import threading
import time
import math
import hiwonder.PID as PID
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC

class AutoShootNode(Node):
    def __init__(self):
        super().__init__('auto_shoot_node')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.ball_info_pub = self.create_publisher(String, 'ball_info', 10)
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return

        self.board = rrc.Board()
        self.ctl = Controller(self.board)
        self.servo_data = {'servo1': 891, 'servo2': 1661}
        self.x_dis = self.servo_data['servo2']
        self.y_dis = self.servo_data['servo1']
        
        self.x_pid = PID.PID(P=0.145, I=0.00, D=0.0007)
        self.y_pid = PID.PID(P=0.145, I=0.00, D=0.0007)
        self.target_color = 'red'
        self.size = (320, 240)
        self.center_x = self.size[0] / 2
        self.center_y = self.size[1] / 2
        self.CENTER_X = 350
        
        self.lab_data = {
            'red': {'min': [0, 166, 135], 'max': [255, 255, 255]},
            'green': {'min': [47, 0, 135], 'max': [255, 110, 255]},
            'blue': {'min': [0, 0, 0], 'max': [255, 146, 120]}
        }
        self.range_rgb = {
            'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 255, 0),
            'black': (0, 0, 0), 'white': (255, 255, 255)
        }
        
        self.t1 = 0
        self.d_x = 20
        self.d_y = 20
        self.step = 1
        self.step_ = 1
        self.last_status = ''
        self.start_count = True
        self.CenterX, self.CenterY = -2, -2
        
        self.init_move()
        self.timer = self.create_timer(0.1, self.process_image)

        # === 语音控制门控锁逻辑 ===
        self.task_triggered = False  # 锁住机器人的双腿
        # 注意：这里订阅 '/start_transport' 是为了直接兼容你原有的 Coordinator
        # 如果你更新了 Coordinator 发送 '/start_kick'，请将这里同步修改
        self.trigger_sub = self.create_subscription(
            Bool,
            '/start_kick', 
            self.trigger_callback,
            10)

        # 启动动作控制线程
        self.action_thread = threading.Thread(target=self.move)
        self.action_thread.daemon = True
        self.action_thread.start()
        
        self.get_logger().info("⚽ 自动踢球节点已启动，正在等待语音唤醒指令...")

    # === 新增语音唤醒回调函数 ===
    def trigger_callback(self, msg):
        if msg.data and not self.task_triggered:
            self.get_logger().info("🎤 收到唤醒指令！解除双腿锁定，开始踢球！")
            self.task_triggered = True

    def init_move(self):
        self.ctl.set_pwm_servo_pulse(1, self.servo_data['servo1'], 500)
        self.ctl.set_pwm_servo_pulse(2, self.servo_data['servo2'], 500)
        self.get_logger().info("舵机初始化完成")

    def get_area_max_contour(self, contours):
        max_area = 0
        max_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area and 1000 > area > 2:
                max_area = area
                max_contour = c
        return max_contour, max_area

    def ball_detect(self, img):
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(frame_lab,
                          tuple(self.lab_data[self.target_color]['min']),
                          tuple(self.lab_data[self.target_color]['max']))
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        area_max_contour, area_max = self.get_area_max_contour(contours)
        
        if area_max:
            try:
                (self.CenterX, self.CenterY), radius = cv2.minEnclosingCircle(area_max_contour)
                self.CenterX = int(self.CenterX * img_w / self.size[0])
                self.CenterY = int(self.CenterY * img_h / self.size[1])
                radius = int(radius * img_w / self.size[0])
                use_time = 0
                if self.y_dis == self.servo_data['servo1'] and abs(self.x_dis - self.servo_data['servo2']) < 150:
                    self.x_dis = self.servo_data['servo2']
                else:
                    self.x_pid.SetPoint = img_w / 2
                    self.x_pid.update(self.CenterX)
                    d_x = int(self.x_pid.output)
                    self.last_status = 'left' if d_x > 0 else 'right'
                    use_time = abs(d_x * 0.00025)
                    self.x_dis += d_x
                    self.x_dis = max(self.servo_data['servo2'] - 400, min(self.servo_data['servo2'] + 400, self.x_dis))
                
                self.y_pid.SetPoint = img_h / 2
                self.y_pid.update(self.CenterY)
                d_y = int(self.y_pid.output)
                use_time = round(max(use_time, abs(d_y * 0.00025)), 5)
                self.y_dis += d_y
                self.y_dis = max(self.servo_data['servo1'], min(1200, self.y_dis))
                self.ctl.set_pwm_servo_pulse(1, self.y_dis, int(use_time * 1000))
                self.ctl.set_pwm_servo_pulse(2, self.x_dis, int(use_time * 1000))
                time.sleep(use_time)
                
                cv2.circle(img, (self.CenterX, self.CenterY), radius, self.range_rgb[self.target_color], 2)
                cv2.line(img, (int(self.CenterX - radius/2), self.CenterY), (int(self.CenterX + radius/2), self.CenterY), self.range_rgb[self.target_color], 2)
                cv2.line(img, (self.CenterX, int(self.CenterY - radius/2)), (self.CenterX, int(self.CenterY + radius/2)), self.range_rgb[self.target_color], 2)
            except Exception as e:
                self.get_logger().error(f"球检测出错: {e}")
                self.CenterX, self.CenterY = -1, -1
        else:
            self.CenterX, self.CenterY = -1, -1
        return img

    def move(self):
        while True:
            if rclpy.ok():
                # === 拦截器：如果未收到语音唤醒信号，保持站立不动 ===
                if not self.task_triggered:
                    time.sleep(0.1)
                    continue

                # 原有状态机逻辑
                if self.CenterX >= 0:
                    self.step_ = 1
                    self.d_x, self.d_y = 20, 20
                    self.start_count = True
                    if self.step == 1:
                        if self.x_dis - self.servo_data['servo2'] > 150:
                            AGC.runActionGroup('turn_left_small_step')
                        elif self.x_dis - self.servo_data['servo2'] < -150:
                            AGC.runActionGroup('turn_right_small_step')
                        else:
                            self.step = 2
                    elif self.step == 2:
                        if self.y_dis == self.servo_data['servo1']:
                            if self.x_dis == self.servo_data['servo2'] - 400:
                                AGC.runActionGroup('turn_right', times=2)
                            elif self.x_dis == self.servo_data['servo2'] + 400:
                                AGC.runActionGroup('turn_left', times=2)
                            elif 350 < self.CenterY <= 380:
                                AGC.runActionGroup('go_forward_one_step')
                                self.last_status = 'go'
                                self.step = 1
                            elif 120 < self.CenterY <= 350:
                                AGC.runActionGroup('go_forward')
                                self.last_status = 'go'
                                self.step = 1
                            elif 0 <= self.CenterY <= 120 and abs(self.x_dis - self.servo_data['servo2']) <= 200:
                                AGC.runActionGroup('go_forward_fast')
                                self.last_status = 'go'
                                self.step = 1
                            else:
                                self.step = 3
                        else:
                            if self.x_dis == self.servo_data['servo2'] - 400:
                                AGC.runActionGroup('turn_right', times=2)
                            elif self.x_dis == self.servo_data['servo2'] + 400:
                                AGC.runActionGroup('turn_left', times=2)
                            else:
                                AGC.runActionGroup('go_forward_fast')
                                self.last_status = 'go'
                    elif self.step == 3:
                        if self.y_dis == self.servo_data['servo1']:
                            if abs(self.CenterX - self.CENTER_X) <= 40:
                                AGC.runActionGroup('left_move')
                            elif 0 < self.CenterX < self.CENTER_X - 90:
                                AGC.runActionGroup('left_move_fast')
                                time.sleep(0.2)
                            elif self.CENTER_X + 90 < self.CenterX:
                                AGC.runActionGroup('right_move_fast')
                                time.sleep(0.2)
                            else:
                                self.step = 4
                        else:
                            if 270 <= self.x_dis - self.servo_data['servo2'] < 480:
                                AGC.runActionGroup('left_move_fast')
                                time.sleep(0.2)
                            elif abs(self.x_dis - self.servo_data['servo2']) < 170:
                                AGC.runActionGroup('left_move')
                            elif -480 < self.x_dis - self.servo_data['servo2'] <= -270:
                                AGC.runActionGroup('right_move_fast')
                                time.sleep(0.2)
                            else:
                                self.step = 4
                    elif self.step == 4:
                        if self.y_dis == self.servo_data['servo1']:
                            if 380 < self.CenterY <= 440:
                                AGC.runActionGroup('go_forward_one_step')
                                self.last_status = 'go'
                            elif 0 <= self.CenterY <= 380:
                                AGC.runActionGroup('go_forward')
                                self.last_status = 'go'
                            else:
                                if self.CenterX < self.CENTER_X:
                                    AGC.runActionGroup('left_shot_fast')
                                else:
                                    AGC.runActionGroup('right_shot_fast')
                                self.step = 1
                        else:
                            self.step = 1
                elif self.CenterX == -1:
                    if self.last_status == 'go':
                        self.last_status = ''
                        AGC.runActionGroup('back_fast', with_stand=True)
                    elif self.start_count:
                        self.start_count = False
                        self.t1 = time.time()
                    else:
                        if time.time() - self.t1 > 0.5:
                            if self.step_ == 5:
                                self.x_dis += self.d_x
                                if abs(self.x_dis - self.servo_data['servo2']) <= abs(self.d_x):
                                    AGC.runActionGroup('turn_right')
                                    self.step_ = 1
                            if self.step_ == 1 or self.step_ == 3:
                                self.x_dis += self.d_x
                                if self.x_dis > self.servo_data['servo2'] + 400:
                                    if self.step_ == 1:
                                        self.step_ = 2
                                    self.d_x = -self.d_x
                                elif self.x_dis < self.servo_data['servo2'] - 400:
                                    if self.step_ == 3:
                                        self.step_ = 4
                                    self.d_x = -self.d_x
                            elif self.step_ == 2 or self.step_ == 4:
                                self.y_dis += self.d_y
                                if self.y_dis > 1200:
                                    if self.step_ == 2:
                                        self.step_ = 3
                                    self.d_y = -self.d_y
                                elif self.y_dis < self.servo_data['servo1']:
                                    if self.step_ == 4:
                                        self.step_ = 5
                                    self.d_y = -self.d_y
                            self.ctl.set_pwm_servo_pulse(1, self.y_dis, 20)
                            self.ctl.set_pwm_servo_pulse(2, self.x_dis, 20)
                            time.sleep(0.02)
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

    def process_image(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        frame = self.ball_detect(frame)
        ball_msg = String()
        ball_msg.data = f"Ball Center: ({self.CenterX}, {self.CenterY})"
        self.ball_info_pub.publish(ball_msg)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = AutoShootNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()