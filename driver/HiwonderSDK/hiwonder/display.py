import time
import Hobot.GPIO as GPIO
from hiwonder.number_model import render_number
# 定义TM1640命令
TM1640_CMD1 = 0x40  # 数据命令
TM1640_CMD2 = 0xC0  # 地址命令
TM1640_CMD3 = 0x80  # 显示控制命令
TM1640_DSP_ON = 0x08  # 显示开启
TM1640_DELAY = 10   # 10us 延迟

def sleep_us(t):
    time.sleep(t / 1000000)

class TM1640(object):
    digit_dict = {
        '0': 0x3f, '1': 0x06, '2': 0x5b, '3': 0x4f,
        '4': 0x66, '5': 0x6d, '6': 0x7d, '7': 0x07,
        '8': 0x7f, '9': 0x6f, '.': 0x80, '-': 0x40
    }

    def __init__(self, clk, dio, brightness=7):
        if not 0 <= brightness <= 7:
            raise ValueError("亮度超出范围")
        self._brightness = brightness
        self.display_buf = [0] * 16
        self.clk = clk
        self.dio = dio
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(clk, GPIO.OUT)
        GPIO.setup(dio, GPIO.OUT)
        GPIO.output(self.clk, 0)
        GPIO.output(self.dio, 0)
        sleep_us(TM1640_DELAY)
        self._write_data_cmd()
        self._write_dsp_ctrl()

    def _start(self):
        GPIO.output(self.dio, 0)
        sleep_us(TM1640_DELAY)
        GPIO.output(self.clk, 0)
        sleep_us(TM1640_DELAY)

    def _stop(self):
        GPIO.output(self.dio, 0)
        sleep_us(TM1640_DELAY)
        GPIO.output(self.clk, 1)
        sleep_us(TM1640_DELAY)
        GPIO.output(self.dio, 1)

    def _write_data_cmd(self):
        self._start()
        self._write_byte(TM1640_CMD1)
        self._stop()

    def _write_dsp_ctrl(self):
        self._start()
        self._write_byte(TM1640_CMD3 | TM1640_DSP_ON | self._brightness)
        self._stop()

    def _write_byte(self, b):
        for i in range(8):
            GPIO.output(self.dio, (b >> i) & 1)
            sleep_us(TM1640_DELAY)
            GPIO.output(self.clk, 1)
            sleep_us(TM1640_DELAY)
            GPIO.output(self.clk, 0)
            sleep_us(TM1640_DELAY)

    def brightness(self, val=None):
        """设置显示亮度 0-7。"""
        if val is None:
            return self._brightness
        if not 0 <= val <= 7:
            raise ValueError("亮度超出范围")

        self._brightness = val
        self._write_data_cmd()
        self._write_dsp_ctrl()

    def write(self, rows, pos=0):
        if not 0 <= pos <= 7:
            raise ValueError("位置超出范围")

        self._write_data_cmd()
        self._start()

        self._write_byte(TM1640_CMD2 | pos)
        for row in rows:
            self._write_byte(row)

        self._stop()
        self._write_dsp_ctrl()

    def write_int(self, int_val, pos=0, len=8):
        self.write(int_val.to_bytes(len, 'big'), pos)

    def write_hmsb(self, buf, pos=0):
        self._write_data_cmd()
        self._start()

        self._write_byte(TM1640_CMD2 | pos)
        for i in range(7 - pos, -1, -1):
            self._write_byte(buf[i])

        self._stop()
        self._write_dsp_ctrl()

    def clear(self):
        """熄灭点阵所有LED。"""
        self.display_buf = [0] * 16
        self.update_display()

    def set_bit(self, x, y, s):
        """设置(x, y)处的LED状态，0或者1，0表示熄灭。"""
        self.display_buf[x] = (self.display_buf[x] & (~(0x01 << y))) | (s << y)

    def set_number(self, number):
        """数码管显示数字。"""
        self.display_buf = [self.digit_dict['0']] * 4
        num = list(str(number))
        num.reverse()
        for i in range(len(num)):
            self.display_buf[-i-1] = self.digit_dict[num[i]]
            if num[i] == '.':
                self.display_buf[-i-2] = self.digit_dict[num[i - 1]] + self.digit_dict['.']

    def set_buf_horizontal(self, buf):
        """设置显示的二进制列表，横向。"""
        l = zip(*buf[::-1])
        self.display_buf = [int(''.join(i), 2) for i in l]

    def set_buf_vertical(self, buf):
        """设置显示的二进制列表，竖向。"""
        self.display_buf = [int(i, 2) for i in buf]

    def update_display(self):
        """刷新显示。"""
        self.write(self.display_buf)

if __name__ == "__main__":
    try:
        display = TM1640(dio=22, clk=24)
        display.clear()
        while True:
            display.display_buf = render_number("1", "2", "%", "%")
            print(display.display_buf)
            display.update_display()
            time.sleep(1)
    except KeyboardInterrupt:
        display.clear()
        GPIO.cleanup()  # 清理GPIO设置
