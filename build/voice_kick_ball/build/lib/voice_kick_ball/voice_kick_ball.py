import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import os  # 新增：用于调用 Linux 系统级命令彻底关闭进程

class VoiceKickCoordinator(Node):
    def __init__(self):
        # 节点名更新，保持语义清晰
        super().__init__('voice_kick_coordinator')
        self.get_logger().info("🧠 语音-踢球协调中枢已启动，正在待命...")

        # 1. 听觉接收：订阅上游 ASR 节点发来的文本指令
        self.text_sub = self.create_subscription(
            String,
            '/text_input',
            self.text_callback,
            10)

        # 2. 动作下发：专用话题改为 /start_kick，避免和以前的搬运代码冲突
        self.trigger_pub = self.create_publisher(
            Bool,
            '/start_kick',
            10)

        # 状态锁：保证机器人只会被唤醒一次
        self.task_triggered = False

    def levenshtein_distance(self, s1, s2):
        """计算两个字符串的编辑距离，用于语音指令容错"""
        if len(s1) < len(s2):
            return self.levenshtein_distance(s2, s1)
        if len(s2) == 0:
            return len(s1)
        previous_row = range(len(s2) + 1)
        for i, c1 in enumerate(s1):
            current_row = [i + 1]
            for j, c2 in enumerate(s2):
                insertions = previous_row[j + 1] + 1
                deletions = current_row[j] + 1
                substitutions = previous_row[j] + (c1 != c2)
                current_row.append(min(insertions, deletions, substitutions))
            previous_row = current_row
        return previous_row[-1]

    def text_callback(self, msg):
        # 【修改点 1】如果踢球任务已经开始，直接无视后续的所有声音
        if self.task_triggered:
            return 

        command = msg.data.strip()
        self.get_logger().info(f"👂 听到声音: {command}")

        # 【修改点 2】优化目标词：你在测试中经常只说“开始”，所以目标词定为“开始”最稳妥
        target_cmd = "开始"
        
        # 核心逻辑：允许误差，或者指令中包含了目标词
        if self.levenshtein_distance(command, target_cmd) <= 2 or target_cmd in command:
            self.get_logger().info(f"✅ 匹配到指令 [{command}]！正在向双腿下发 [踢球] 启动信号...")
            
            # 组装一个布尔值消息
            trigger_msg = Bool()
            trigger_msg.data = True
            
            # 向 /start_kick 话题开火！
            self.trigger_pub.publish(trigger_msg)
            self.task_triggered = True  # 锁死中枢，不再重复下发
            
            # 【修改点 3】物理级“关闭耳朵”：直接杀掉 ASR 节点进程！
            self.get_logger().info("🛑 指令已下发！正在彻底终结录音程序，让机器人专心踢球...")
            os.system("pkill -f asr_node")  # 这行代码会在 Linux 后台直接秒杀录音节点