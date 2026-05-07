#!/usr/bin/env python3
# encoding:utf-8
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import threading
import time
import math

import hiwonder.ActionGroupControl 
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.PID as PID

class ColorTrackNode(Node):
    def __init__(self):
        super().__init__('color_track_kf')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.color_info_pub = self.create_publisher(String, 'color_info', 10)
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("无法打开摄像头")
            return
        
        # =========================================================================
        # 【硬件底层初始化】
        # =========================================================================
        self.board = rrc.Board(device="/dev/ttyS1", baudrate=1000000)
        self.board.enable_reception()
        
        self.ctl = Controller(self.board)
        self.servo_data = {'servo1': 1500, 'servo2': 1500}
        self.x_dis, self.y_dis = self.servo_data['servo2'], self.servo_data['servo1']
        
        self.x_pid = PID.PID(P=0.25, I=0.00, D=0.0015)
        self.y_pid = PID.PID(P=0.25, I=0.00, D=0.0015)
        
        self.current_gx, self.current_gy, self.current_gz = 0.0, 0.0, 0.0
        self.k_yaw = 2.5    
        self.k_pitch = 2.0  
        self.smoothed_x_dis = self.x_dis
        self.smoothed_y_dis = self.y_dis

        self.current_radius = 0

        # =========================================================================
        # 🤖 【状态机配置】 
        # =========================================================================
        self.state = 'INIT'  
        self.last_state = 'INIT'
        self.lost_counter = 0    
        self.scan_start_time = 0 
        self.init_start_time = 0 
        self.lock_counter = 0  
        self.is_action_running = False     
        
        self.CENTER_X = 1500
        self.CENTER_Y = 1500  
        self.HEAD_LEFT_LIMIT = 2000
        self.HEAD_RIGHT_LIMIT = 1000
        self.TURN_360_TIME = 25.0  

        # =========================================================================
        # 【颜色与卡尔曼配置】
        # =========================================================================
        self.target_color = 'red'  
        self.size = (320, 240)
        self.lab_data = {
            'red': {'min': [0, 166, 135], 'max': [255, 255, 255]},
            'green': {'min': [47, 0, 135], 'max': [255, 110, 255]},
            'blue': {'min': [0, 0, 0], 'max': [255, 146, 120]}
        }
        self.range_rgb = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}
        
        self.kf = cv2.KalmanFilter(4, 2)
        self.is_tracking = False 
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        R_value = 1e-2
        self.kf.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * R_value
        Q_pos_value = 1e-1
        Q_vel_value = 1e-2
        self.kf.processNoiseCov = np.array([[Q_pos_value, 0, 0, 0], [0, Q_pos_value, 0, 0], [0, 0, Q_vel_value, 0], [0, 0, 0, Q_vel_value]], np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 1.0

        self.init_move()
        
        self.sampling_freq = 0.03
        self.timer = self.create_timer(self.sampling_freq, self.process_image)
        
        self.servo_thread = threading.Thread(target=self.servo_control, daemon=True)
        self.servo_thread.start()
        
        self.imu_thread = threading.Thread(target=self.imu_read_thread, daemon=True)
        self.imu_thread.start()
        
        self.get_logger().info(f"🔴 具身智能追踪系统就绪！不仅能转圈找，还能前后跟随！")

    def init_move(self):
        self.get_logger().info("⏳ 正在执行硬件唤醒 (Stand)...")
        hiwonder.ActionGroupControl.runActionGroup('stand')
        time.sleep(2.0) 
        self.ctl.set_pwm_servo_pulse(1, self.y_dis, 500)
        self.ctl.set_pwm_servo_pulse(2, self.x_dis, 500)
        self.get_logger().info("✅ 唤醒完毕，释放控制权！")

    def imu_read_thread(self):
        while rclpy.ok():
            imu_data = self.board.get_imu()
            if imu_data is not None:
                ax, ay, az, gx, gy, gz = imu_data
                self.current_gx = gx
                self.current_gy = gy
                self.current_gz = gz
            time.sleep(0.01) 

    def execute_action_async(self, action_name):
        self.is_action_running = True
        try:
            hiwonder.ActionGroupControl.runActionGroup(action_name)
            time.sleep(0.2) 
        except Exception as e:
            self.get_logger().error(f"底盘动作出错: {e}")
        finally:
            self.is_action_running = False

    def get_area_max_contour(self, contours):
        max_area = 0
        max_contour = None
        for c in contours:
            area = math.fabs(cv2.contourArea(c))
            if area > max_area and area > 100:
                max_area = area
                max_contour = c
        return max_contour, max_area

    def color_detect(self, img):
        img_h, img_w = img.shape[:2]
        
        cv2.line(img, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
        cv2.line(img, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)
        
        prediction = self.kf.predict()
        
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        
        mask = cv2.inRange(frame_lab, tuple(self.lab_data[self.target_color]['min']), tuple(self.lab_data[self.target_color]['max']))
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        area_max_contour, area_max = self.get_area_max_contour(contours)
        
        center_x, center_y, radius = -1, -1, 0
        
        if area_max > 300:
            (center_x_small, center_y_small), radius_small = cv2.minEnclosingCircle(area_max_contour)
            meas_x = int(center_x_small * img_w / self.size[0])
            meas_y = int(center_y_small * img_h / self.size[1])
            radius = int(radius_small * img_w / self.size[0])
            
            self.current_radius = radius
            
            cv2.circle(img, (meas_x, meas_y), radius, (0, 255, 255), 2)
            measurement = np.array([[meas_x], [meas_y]], dtype=np.float32)
            
            if not self.is_tracking:
                self.kf.statePost = np.array([[meas_x], [meas_y], [0], [0]], dtype=np.float32)
                self.kf.statePre = self.kf.statePost
                self.is_tracking = True
                filter_x, filter_y = meas_x, meas_y
            else:
                estimated = self.kf.correct(measurement)
                filter_x, filter_y = int(estimated[0][0]), int(estimated[1][0])
            
            cv2.circle(img, (filter_x, filter_y), 5, self.range_rgb[self.target_color], -1)
            
            if abs(filter_x - img_w / 2.0) < 15:
                filter_x = img_w / 2.0
            self.x_pid.SetPoint = img_w / 2
            self.x_pid.update(filter_x)
            step_x = max(-30, min(30, int(self.x_pid.output)))
            self.x_dis += step_x
            self.x_dis = max(500, min(2500, self.x_dis))
            
            if abs(filter_y - img_h / 2.0) < 15:
                filter_y = img_h / 2.0
            self.y_pid.SetPoint = img_h / 2
            self.y_pid.update(filter_y)
            step_y = max(-30, min(30, int(self.y_pid.output)))
            self.y_dis += step_y
            self.y_dis = max(1000, min(2000, self.y_dis))
            
            center_x, center_y = filter_x, filter_y
        else:
            self.is_tracking = False
            self.current_radius = 0 
            
        return center_x, center_y, radius

    def servo_control(self):
        """主状态机线程"""
        self.init_start_time = time.time()
        
        while True:
            if rclpy.ok():
                use_time = 0.03
                
                if self.state == 'INIT':
                    if time.time() - self.init_start_time > 2.0:
                        self.state = 'HEAD_SCAN'
                        self.scan_start_time = time.time()
                
                elif self.is_tracking:
                    self.lost_counter = 0
                    if abs(self.x_dis - self.CENTER_X) > 150: 
                        self.state = 'ALIGN'
                    else:
                        self.state = 'TRACKING'
                else:
                    if self.state in ['TRACKING', 'ALIGN']:
                        self.lost_counter += 1
                        if self.lost_counter > 50:
                            self.state = 'HEAD_SCAN'
                            self.scan_start_time = time.time()
                
                if self.state != self.last_state:
                    self.get_logger().info(f"🔄 战术切换: {self.last_state} ---> {self.state}")
                    self.last_state = self.state

                if self.state == 'TRACKING':
                    if not self.is_action_running:
                        if self.current_radius > 100: 
                            self.get_logger().info("😨 目标太近！战术后退...")
                            threading.Thread(target=self.execute_action_async, args=('back_fast',), daemon=True).start()
                            self.lock_counter = 0
                        elif 0 < self.current_radius < 30: 
                            self.get_logger().info("🏃‍♂️ 目标太远！小步跟进...")
                            threading.Thread(target=self.execute_action_async, args=('go_forward',), daemon=True).start()
                            self.lock_counter = 0
                        else:
                            if abs(self.x_dis - self.CENTER_X) < 60 and abs(self.y_dis - self.CENTER_Y) < 60:
                                self.lock_counter += 1
                                if self.lock_counter > 60:
                                    self.get_logger().info("🎉 完美锁定！开心庆祝~")
                                    threading.Thread(target=self.execute_action_async, args=('chest',), daemon=True).start()
                                    self.lock_counter = 0 
                            else:
                                self.lock_counter = 0
                    
                elif self.state == 'ALIGN':
                    if not self.is_action_running:
                        if self.x_dis > (self.CENTER_X + 100):
                            threading.Thread(target=self.execute_action_async, args=('turn_left',), daemon=True).start()
                        elif self.x_dis < (self.CENTER_X - 100):
                            threading.Thread(target=self.execute_action_async, args=('turn_right',), daemon=True).start()

                elif self.state == 'HEAD_SCAN':
                    elapsed = time.time() - self.scan_start_time
                    scan_cycle = 4.0  
                    
                    if elapsed < scan_cycle:
                        offset_x = math.sin((elapsed / scan_cycle) * 2 * math.pi) * 500
                        offset_y = math.sin((elapsed / (scan_cycle / 2)) * 2 * math.pi) * 300
                        self.x_dis = int(self.CENTER_X + offset_x)
                        self.y_dis = int(self.CENTER_Y + offset_y)
                    else:
                        self.state = 'BODY_SCAN'
                        self.scan_start_time = time.time()

                elif self.state == 'BODY_SCAN':
                    elapsed = time.time() - self.scan_start_time
                    if elapsed < self.TURN_360_TIME:
                        self.x_dis = self.HEAD_LEFT_LIMIT  
                        self.y_dis = self.CENTER_Y - 300   
                        if not self.is_action_running:
                            threading.Thread(target=self.execute_action_async, args=('turn_left',), daemon=True).start()
                    else:
                        self.state = 'HEAD_SCAN'
                        self.scan_start_time = time.time()
                        self.lost_counter = 0

                yaw_comp = int(self.current_gz * self.k_yaw)
                pitch_comp = int(self.current_gy * self.k_pitch)
                
                raw_target_x = max(500, min(2500, self.x_dis + yaw_comp))
                raw_target_y = max(1000, min(2000, self.y_dis + pitch_comp))
                
                alpha = 1.0
                self.smoothed_x_dis = alpha * raw_target_x + (1.0 - alpha) * self.smoothed_x_dis
                self.smoothed_y_dis = alpha * raw_target_y + (1.0 - alpha) * self.smoothed_y_dis
                
                self.ctl.set_pwm_servo_pulse(1, int(self.smoothed_y_dis), int(use_time * 1000))
                self.ctl.set_pwm_servo_pulse(2, int(self.smoothed_x_dis), int(use_time * 1000))
                
                time.sleep(use_time)
            else:
                time.sleep(0.01)

    def process_image(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        center_x, center_y, radius = self.color_detect(frame)
        
        color_msg = String()
        color_msg.data = f"Color:{self.target_color}, KF_X:{center_x}, KF_Y:{center_y}, R:{radius}"
        self.color_info_pub.publish(color_msg)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    # 🚀 核心改动 1：新增专用的安全退出函数
    def cleanup(self):
        self.get_logger().info("🛑 接收到退出指令，正在执行硬件安全复位...")
        
        # 1. 摄像头云台强行回正
        self.ctl.set_pwm_servo_pulse(1, self.CENTER_Y, 500)
        self.ctl.set_pwm_servo_pulse(2, self.CENTER_X, 500)
        
        # 2. 强行下发站立指令，打断其他动作
        try:
            hiwonder.ActionGroupControl.runActionGroup('stand')
            time.sleep(0.5) # 给舵机复位留出时间
        except Exception as e:
            self.get_logger().error(f"底盘复位出错: {e}")
            
        # 3. 安全释放摄像头硬件，防止设备占用卡死
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        
        self.get_logger().info("✅ 硬件复位完毕，已安全退出。")

def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackNode()
    try:
        rclpy.spin(node)
    # 🚀 核心改动 2：精确捕捉用户的 Ctrl+C 中断信号
    except KeyboardInterrupt:
        node.get_logger().info("键盘中断 (Ctrl+C) 被触发！")
    finally:
        # 🚀 核心改动 3：不论程序是正常结束还是被强杀，必须先执行 cleanup 再销毁节点
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()