import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class VoiceTransportCoordinator(Node):
    def __init__(self):
        super().__init__('voice_transport_coordinator')
        self.get_logger().info("🧠 语音-运输协调节点已启动，正在待命...")

        # 1. 听觉接收：订阅上游 ASR 节点发来的文本指令
        self.text_sub = self.create_subscription(
            String,
            '/text_input',
            self.text_callback,
            10)

        # 2. 动作下发：创建一个新的话题发布者，专门用来唤醒运输节点
        self.trigger_pub = self.create_publisher(
            Bool,
            '/start_transport',
            10)

        # 状态锁：防止重复发送启动信号
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
        command = msg.data.strip()
        self.get_logger().info(f"👂 听到声音: {command}")

        if self.task_triggered:
            # 如果已经唤醒了机器人，就不要再重复打扰它干活了
            return 

        target_cmd = "开始踢球"
        
        # 核心逻辑：允许最多 2 个字的识别误差，或者指令中包含了目标词
        if self.levenshtein_distance(command, target_cmd) <= 2 or target_cmd in command:
            self.get_logger().info(f"✅ 匹配到指令 [{command}]！正在向底层下发启动信号...")
            
            # 组装一个布尔值消息
            trigger_msg = Bool()
            trigger_msg.data = True
            
            # 向 /start_transport 话题开火！
            self.trigger_pub.publish(trigger_msg)
            self.task_triggered = True

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTransportCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程序被用户中断(Ctrl+C)，正在安全退出...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()