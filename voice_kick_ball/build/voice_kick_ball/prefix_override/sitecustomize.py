import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/userdata/dev_ws/src/originman/voice_kick_ball/install/voice_kick_ball'
