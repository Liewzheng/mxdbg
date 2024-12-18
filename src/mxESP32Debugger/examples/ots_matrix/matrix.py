from mxESP32Debugger.debugger import Dbg
from loguru import logger
import marimo as mo
import numpy as np

def matrix_init(dev:Dbg, int_pin:int = 40, reset_pin:int = 39):
    
    dev.gpio_config(pin=reset_pin, mode=2, pull_up=True, pull_down=False)
    dev.gpio_config(pin=int_pin, mode=1, pull_up=False, pull_down=False)
    dev.gpio_write_read(pin=reset_pin, level=1)
    
    _, slave_list = dev.i2c_find_slave()  # 查找设备，如果关键控制设备不在则报错
    assert "0x18" in slave_list and "0x20" in slave_list, f"i2c find slave failed: {slave_list}"
    
    reg_list = [
        [0x02, 0x00],  # set all pins as normal polarity
        [0x03, 0x00],  # set all pins as output mode
        [0x01, 0x00],  # set all pins as low level
    ]

    for reg in reg_list:
        ret, _ = dev.i2c_write_read(slave_id=0x18, write_list=reg, read_length=0)
        assert ret, f"write {reg} to 0x18 failed"
        
        ret, data = dev.i2c_write_read(slave_id=0x18, write_list=[reg[0]], read_length=1)
        assert ret, f"read {reg[0]} from 0x18 failed"
        assert data[0] == reg[1], f"read {reg[0]} from 0x18 failed"

    reg_list = [
        [0x04, 0x00, 0x00],  # disable polarity inversion
        [0x06, 0x00, 0x00],  # set all pins as output mode
        [0x02, 0x00, 0x00],  # set all pins as low level
    ]

    for reg in reg_list:
        ret, _ = dev.i2c_write_read(slave_id=0x20, write_list=reg, read_length=0)
        assert ret, f"write {reg} to 0x20 failed"
        
        ret, data = dev.i2c_write_read(slave_id=0x20, write_list=[reg[0]], read_length=2)
        assert ret, f"read {reg[0]} from 0x20 failed"
        assert data[0] == reg[1], f"read {reg[0]} from 0x20 failed"
        assert data[1] == reg[2], f"read {reg[0]} from 0x20 failed"

def matrix_power_control(dev:Dbg, x: int = 0, y: int = 0, on: bool = False):
    """
    @brief using x and y to control the matrix power
    @param x: 0 ~ 7
    @param y: 0 ~ 7
    @param on: True or False
    """

    """
    x and y should be 0 ~ 7
    """

    if x < 0 or x > 7 or y < 0 or y > 7:
        logger.error("x and y should be 0 ~ 7")
        return False

    if on is not True:
        return False

    # coloum 0 ~ 1 (16 chips in total) controlled by IO00 ~ IO03(TCA9555) and EN0(PCA9557)
    # coloum 2 ~ 3 (16 chips in total) controlled by IO04 ~ IO07(TCA9555) and EN1(PCA9557)
    # column 4 ~ 5 (16 chips in total) controlled by IO10 ~ IO13(TCA9555) and EN2(PCA9557)
    # column 6 ~ 7 (16 chips in total) controlled by IO14 ~ IO17(TCA9555) and EN3(PCA9557)

    """
    enable_pin range is [0, 3] (EN0 / EN1 / EN2 / EN3), which is controlled by PCA9557PW
    each enable pin take the charge of 2 ADG1606BRUZ chips
    (LDP00~LDP07 / LDP10~LDP17 / VDD00~VDD07 / VDD10~VDD17)

    Though there are 8 ADG1606BRUZ chips in total, but only 2 of them will be active in
    the same time by controlling the IO pins of TCA9555PWR.

    """

    enable_pin = x // 2

    column = x % 2  # column range is [0, 1]

    bitmask = 0x000F << (enable_pin * 4)  # 0xF000, 0x0F00, 0x00F0, 0x000F
    bitmask_ = column * 8 + y  # bitmask_ range is [0, 15]
    bitmask &= bitmask_ << (enable_pin * 4)

    # print(
    #     f"TCA9555PWR (0x20) enable_pin {enable_pin}, column {column}, bitmask {bitmask >> 8 :08b} {bitmask & 0x00FF :08b}"
    # )
    
    loop_counts = 0
    while True:
        ret, _ = dev.i2c_write_read(
            slave_id=0x20, write_list=[0x02, (bitmask & 0x00FF), (bitmask & 0xFF00) >> 8], read_length=0
        )
        loop_counts += 1
        if ret is True:
            break
        if loop_counts > 10:
            logger.error(f"write 0x02,0x{bitmask & 0x00FF:02X},0x{(bitmask & 0xFF00) >> 8:02X} to 0x20 failed")
            return False

    """
    the ADG1606BRUZ chips are controlled by PCA9557PW
    by writting IO1 ~ IO4 (EN0 ~EN3) to control the enable pin of ADG1606BRUZ
    """

    en_bitmask = 0x00
    en_bitmask |= 1 << (enable_pin + 1)

    # print(f"PCA9557PW (0x18) en_bitmask {en_bitmask:08b}")

    loop_counts = 0
    while True:
        ret, _ = dev.i2c_write_read(slave_id=0x18, write_list=[0x01, en_bitmask], read_length=0)
        loop_counts += 1
        if ret is True:
            break
        if loop_counts > 10:
            logger.error(f"write 0x01,0x{en_bitmask:02X} to 0x18 failed")
            return False

    loop_counts = 0
    while True:
        ret, data = dev.i2c_write_read(slave_id=0x18, write_list=[0x01], read_length=1)
        loop_counts += 1
        if ret is True:
            break
        if loop_counts > 10:
            logger.error("read data from 0x18 failed")
            return False
    
    if data[0] != en_bitmask:
        print("data[0] != en_bitmask")
        return False
    else:
        return True
    
def gradient_color(start_color, end_color, steps):
    # 分解起始颜色和终止颜色的 RGB 值
    start_rgb = tuple(int(start_color[i : i + 2], 16) for i in (1, 3, 5))
    end_rgb = tuple(int(end_color[i : i + 2], 16) for i in (1, 3, 5))

    # 生成过渡颜色列表
    gradient = []
    for step in range(steps):
        r = int(start_rgb[0] + (end_rgb[0] - start_rgb[0]) * step / (steps - 1))
        g = int(start_rgb[1] + (end_rgb[1] - start_rgb[1]) * step / (steps - 1))
        b = int(start_rgb[2] + (end_rgb[2] - start_rgb[2]) * step / (steps - 1))
        gradient.append(f"#{r:02X}{g:02X}{b:02X}")

    # 所有颜色往后推，第一个颜色改成 #CCFFCC
    gradient = ["#CCFFCC"] + gradient[:-1]
    return gradient


# 定义单元格内容和大小
def matrix_display(failed_data, test_data, start_x: int, start_y: int, end_x: int, end_y: int):
    steps = 64
    colors = gradient_color("#FFC0C0", "#990000", steps)
    rows = []

    test_counts_max = np.max(test_data)

    failed_data_sorted = np.sort(np.unique(failed_data))

    for x in range(start_x, end_x + 1):
        cols = []
        for y in range(start_y, end_y + 1):

            # 匹配颜色
            if test_data[x, y] == 0: # 如果测试次数为0
                color_index = 0
            else:
                if failed_data[x, y] == 0: # 如果测试失败次数为0
                    color_index = 0
                else:
                    color_index = failed_data_sorted.tolist().index(failed_data[x,y])
                    
            hex_color = colors[color_index]
            text = f"{failed_data[x,y]} / {test_data[x,y]}"

            # 创建单元格，带有背景颜色和文本
            cell = mo.Html(
                f"<div style='background-color: {hex_color}; width: 125px; height: 40px; display: flex; align-items: center; justify-content: center;'>{text}</div>"
            )
            cols.append(cell)

        # 将一行单元格水平排列
        rows.append(mo.hstack(cols))

    # 将所有行垂直堆叠形成矩阵
    matrix = mo.vstack(rows)

    return matrix