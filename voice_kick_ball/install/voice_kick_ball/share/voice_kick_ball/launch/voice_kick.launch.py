from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 启动底层的语音转文字 (ASR) 节点
    asr_node = Node(
        package='originman_llm_chat',
        executable='asr_node',
        name='asr_node',
        output='screen'
    )

    # 2. 启动“大脑”：中枢节点 
    coordinator_node = Node(
        package='voice_kick_ball',
        executable='voice_kick_node',
        name='voice_kick_coordinator',
        output='screen'
    )

    # 3. 启动“手脚”：带有门控锁的踢球节点
    kick_node = Node(
        package='voice_kick_ball',
        executable='kick_ball',
        name='kick_node',
        output='screen'
    )

    return LaunchDescription([
        asr_node,
        coordinator_node,
        kick_node
    ])