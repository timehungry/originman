import os
import subprocess
import re

wav_path = os.path.join(os.path.split(os.path.realpath(__file__))[0], 'feedback_voice')



def play(path):
    try:
        os.system(f"aplay -D plughw:0 {path}")

    except BaseException as e:
        print('Error:', e)

