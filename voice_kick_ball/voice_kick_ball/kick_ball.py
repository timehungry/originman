#!/usr/bin/env python3
# encoding:utf-8
import rclpy
from rclpy.node import Node
import cv2
import threading
from flask import Flask, Response
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

import threading
from flask import Flask, Response
import cv2

# ===== Web 视频流专用的全局变量和 Flask 设定 =====
global_frame = None
app = Flask(__name__)

def generate_frames():
    global global_frame
    while True:
        if global_frame is None:
            continue
        # 将 JPEG 画面不断变成数据流推给浏览器
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + global_frame + b'\r\n')

@app.route('/')
def video_feed():
    # 当浏览器访问主页时，返回视频流
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

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
        
        self.x_pid = PID.PID(P=0.25, I=0.00, D=0.0007)
        self.y_pid = PID.PID(P=0.25, I=0.00, D=0.0007)
        self.target_color = 'red'
        self.size = (320, 240)
        self.center_x = self.size[0] / 2
        self.center_y = self.size[1] / 2
        self.CENTER_X = 366
        
        self.lab_data = {
            'red': {'min': [0, 150, 142], 'max': [255, 255, 255]},
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
        self.timer = self.create_timer(0.05, self.process_image)

        # ===== 机体全身初始化 =====
        self.get_logger().info("🧍‍♂️ 正在执行全身立正初始化...")
        # 调用官方动作组：stand
        AGC.runActionGroup('stand') 
        # 给它 1-2 秒时间完成站立动作，防止后续逻辑立刻执行导致重心不稳
        time.sleep(1.5) 
        
        self.get_logger().info("✅ 机器人已站稳，等待语音指令")

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

        # ===== 新增：启动 Flask Web 视频服务器线程 =====
        self.web_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
        self.web_thread.daemon = True
        self.web_thread.start()
        self.get_logger().info("🌐 Web 视频流已启动，请在浏览器访问: http://<机器人IP>:5000")

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
                        # 1. 检查头是否在垂直方向对准（允许 50 脉宽误差，比 10 更稳）
                        if abs(self.y_dis - self.servo_data['servo1']) < 10:
                            
                            # 2. 极限保护：脖子拧不动了就转动身体（改为范围判断）
                            if self.x_dis <= self.servo_data['servo2'] - 380:
                                AGC.runActionGroup('turn_right', times=2)
                                self.step = 1 # 转身后回到第一步重新对准
                            elif self.x_dis >= self.servo_data['servo2'] + 380:
                                AGC.runActionGroup('turn_left', times=2)
                                self.step = 1
                                
                            # 3. 核心：统一慢速接近逻辑
                            # 只要 Y 坐标没超过 365，就一直只走一小步
                            elif self.CenterY <= 365: 
                                self.get_logger().info(f"🚶 接近中 (Y:{self.CenterY})，执行小碎步...")
                                AGC.runActionGroup('go_forward_one_step')
                                self.last_status = 'go'
                                # 💡 关键：走完一小步就回 Step 1 重新看球，这样最稳
                                self.step = 1 
                                
                            else:
                                # 当 CenterY > 365，说明球已经在脚下的黄金位置了
                                self.get_logger().info("🎯 到达射门位置，准备切换到 Step 3 调整脚位")
                                self.step = 3
                                
                        else:
                            # 如果头还没低到位（说明球可能还在远处），也只用慢速挪动
                            if self.x_dis <= self.servo_data['servo2'] - 380:
                                AGC.runActionGroup('turn_right', times=2)
                            elif self.x_dis >= self.servo_data['servo2'] + 380:
                                AGC.runActionGroup('turn_left', times=2)
                            else:
                                AGC.runActionGroup('go_forward_one_step')
                                self.last_status = 'go'
                    elif self.step == 3:
                        # 1. 检查头部是否已低头盯着脚尖 (允许 50 脉宽误差)
                        if abs(self.y_dis - self.servo_data['servo1']) < 10:
                            # 计算当前球心偏离目标的像素误差
                            error_x = self.CenterX - self.CENTER_X
                            
                            # --- 视觉精调：根据像素误差选择步幅 ---
                            if error_x < -90:
                                self.get_logger().info(f"⬅️ 偏左较多({error_x})，执行: left_move_fast")
                                AGC.runActionGroup('left_move_fast')
                                self.step = 1 # 走完回 step 1 重新标定身体方向
                            elif -90 <= error_x < -30:
                                self.get_logger().info(f"⬅️ 接近中心({error_x})，执行: left_move")
                                AGC.runActionGroup('left_move')
                                self.step = 1
                            elif error_x > 90:
                                self.get_logger().info(f"➡️ 偏右较多({error_x})，执行: right_move_fast")
                                AGC.runActionGroup('right_move_fast')
                                self.step = 1
                            elif 30 < error_x <= 90:
                                self.get_logger().info(f"➡️ 接近中心({error_x})，执行: right_move")
                                AGC.runActionGroup('right_move')
                                self.step = 1
                            else:
                                # 落在 [-30, 30] 区间，说明球已经对准了发力脚位
                                self.get_logger().info("🎯 左右位置对齐完成，准备进入 Step 4")
                                self.step = 4
                                
                        # 2. 脖子偏角兜底逻辑 (处理头还没完全低下来的情况)
                        else:
                            neck_offset = self.x_dis - self.servo_data['servo2']
                            # 只要脖子偏角超过 270，就根据方向侧移
                            if neck_offset >= 270:
                                AGC.runActionGroup('left_move_fast')
                            elif neck_offset <= -270:
                                AGC.runActionGroup('right_move_fast')
                            # 脖子基本正了（abs < 170），直接去下一步等视觉精调
                            elif abs(neck_offset) < 170:
                                self.step = 4
                            # 补齐 170-270 的中间区域
                            else:
                                if neck_offset > 0: AGC.runActionGroup('left_move')
                                else: AGC.runActionGroup('right_move')
                                
                            self.step = 1
                    elif self.step == 4:
                        # 只要头低到位了（误差小于 30，比 50 更严谨一点）
                        if abs(self.y_dis - self.servo_data['servo1']) < 30:
                            
                            # 这里的 330 ~ 370 区间完美覆盖了你的黄金值 352
                            if 330 <= self.CenterY <= 375:
                                self.get_logger().info(f"⚽ 黄金位截击！Y:{self.CenterY}, X:{self.CenterX}")
                                
                                # 这里的判断也用 366 作为中轴线
                                if self.CenterX < self.CENTER_X:
                                    AGC.runActionGroup('left_shot_fast')
                                else:
                                    AGC.runActionGroup('right_shot_fast')
                                self.step = 1
                                
                            elif self.CenterY < 330:
                                # 球还没到位，继续挪一小步
                                AGC.runActionGroup('go_forward_one_step')
                                self.step = 1
                                
                        else:
                            # 头还没低到 891 附近，回去继续等
                            self.step = 1
                elif self.CenterX == -1:
                    # 1. 如果刚才正在移动时丢了球，先往后退一步，扩大视野
                    if self.last_status == 'go':
                        self.get_logger().info("⚠️ 移动中丢球，执行 back_fast 尝试找回")
                        self.last_status = ''
                        AGC.runActionGroup('back_fast', with_stand=True)
                        
                    # 2. 初始化计时器（刚丢球时记录当前时间 t1）
                    elif self.start_count:
                        self.start_count = False
                        self.t1 = time.time()
                        
                    # 3. 丢球超过 0.5 秒后，进入“摇头晃脑”搜索模式
                    else:
                        if time.time() - self.t1 > 0.5:
                            # --- 寻球状态机 (step_) ---
                            
                            # Step 1 & 3：左右水平扫描 (Pan)
                            if self.step_ in [1, 3]:
                                self.x_dis += self.d_x
                                # 检查是否到达左右摆头极限（400像素范围）
                                if self.x_dis > self.servo_data['servo2'] + 400:
                                    if self.step_ == 1: self.step_ = 2
                                    self.d_x = -self.d_x # 撞墙回头
                                elif self.x_dis < self.servo_data['servo2'] - 400:
                                    if self.step_ == 3: self.step_ = 4
                                    self.d_x = -self.d_x # 撞墙回头

                            # Step 2 & 4：上下纵向切换 (Tilt)
                            elif self.step_ in [2, 4]:
                                self.y_dis += self.d_y
                                # 检查是否到达仰头或低头极限
                                if self.y_dis > 1200:
                                    if self.step_ == 2: self.step_ = 3
                                    self.d_y = -self.d_y
                                elif self.y_dis < self.servo_data['servo1']:
                                    if self.step_ == 4: self.step_ = 5
                                    self.d_y = -self.d_y

                            # Step 5：全方位找不着，原地转身找
                            elif self.step_ == 5:
                                self.x_dis += self.d_x
                                # 当头回到正前方位置附近时，触发身体旋转
                                if abs(self.x_dis - self.servo_data['servo2']) <= abs(self.d_x):
                                    self.get_logger().info("🕵️ 摇头找不着，原地右转尝试...")
                                    AGC.runActionGroup('turn_right')
                                    self.step_ = 1 # 循环搜索

                            # 4. 实时更新舵机位置
                            self.ctl.set_pwm_servo_pulse(1, self.y_dis, 20) # 俯仰舵机
                            self.ctl.set_pwm_servo_pulse(2, self.x_dis, 20) # 水平舵机
                            time.sleep(0.02)
                            
                        else:
                            # 丢球时间太短，可能只是视觉抖动，原地等 0.01s
                            time.sleep(0.01)
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

    def process_image(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        frame = self.ball_detect(frame)
        
        # ===== 实时显示坐标和舵机数值 (调试神器) =====
        # 1. 显示球坐标 (绿色)
        cv2.putText(frame, f"Ball X: {self.CenterX}  Y: {self.CenterY}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 2. 显示 Y轴 舵机状态 (青色) - 控制俯仰
        text_servo_y = f"Servo Y: {self.y_dis} / Target: {self.servo_data['servo1']}"
        cv2.putText(frame, text_servo_y, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # 3. 显示 X轴 舵机状态 (紫色) - 控制水平
        # 这里加上了你想要的 Servo X
        text_servo_x = f"Servo X: {self.x_dis} / Target: {self.servo_data['servo2']}"
        cv2.putText(frame, text_servo_x, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        # 3. 显示状态误差和步进
        # 计算当前偏差，如果偏差 < 50，文字变绿提示“准许动作”
        error = abs(self.y_dis - self.servo_data['servo1'])
        color_err = (0, 255, 0) if error < 50 else (0, 0, 255)
        cv2.putText(frame, f"Tilt Error: {error}", (10, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_err, 2)

        # 如果当前没有找到球 (CenterX 为 -1)，显示一个醒目的警告
        if self.CenterX < 0:
            cv2.putText(frame, "LOST BALL", (10, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        # ========================================

        # --- 后面保持你原有的逻辑 ---
        ball_msg = String()
        ball_msg.data = f"Ball Center: ({self.CenterX}, {self.CenterY})"
        self.ball_info_pub.publish(ball_msg)
        
        global global_frame
        success, buffer = cv2.imencode('.jpg', frame)
        if success:
            global_frame = buffer.tobytes()
        
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