import marimo

__generated_with = "0.9.32"
app = marimo.App(width="medium")


@app.cell
def __():
    import marimo as mo

    from mxESP32Debugger.debugger import Dbg
    from matrix import matrix_init, matrix_power_control, matrix_display

    import time
    import toml
    import numpy as np

    from tqdm import trange
    return (
        Dbg,
        matrix_display,
        matrix_init,
        matrix_power_control,
        mo,
        np,
        time,
        toml,
        trange,
    )


@app.cell(hide_code=True)
def __(mo):
    mo.md(
        """
        # Matrix 矩阵初始化

        1. 需要根据 Pin Definition ，将 `reset` 口和 `INT` 口配置好，否则可能导致 I2C 从设备 `0x18` (`PCA9557PW`) 通信失败，导致无法进行后续操作。
        2. 需要将所有 IO 扩展芯片（`PCA9557PW` / `TCA9555PWR`）的引脚配置成 **输出**、**拉低** 并且是 **正极性**。

        ## Pin Definition
        | MXB-1031-C1 引脚 | 到ESP32 的默认连接状态 | 描述 |
        |---------------------|------------|------------|
        | 5V | 5V | 给所有芯片供电，包括 控制芯片 和 OTS 芯片 |
        | VCC | / | 单独给控制芯片供电，如果 5V 已经链接，则可以不使用，请置空；<br>如果 5V 不适用，则请连接 VCC|
        | SCL | IIC SCL | |
        | SDA | IIC SDA | |
        | INT | GP40 | IO Expander Interrupt |
        |RESET| GP39 | |
        |VVDD | / | OTS 芯片的供电口，如果左侧 `VDD33` 或 `VDD18` 口跳线帽已连接，则可以不连接。|
        |VLDP | / | 同上 |
        |SC0 | / | OTS 芯片的 I2C，如果 R86/R87 断开，则需要使用|
        |SD0 | / | OTS 芯片的 I2C，如果 R86/R87 断开，则需要使用|
        |MOT | GP37 | |
        |NCS | GP38 | |
        """
    )
    return


@app.cell
def __(Dbg, matrix_init):
    dev = Dbg()
    dev.power_init()
    dev.i2c_config(sda_pullup=False, scl_pullup=False, freq=400_000)
    dev.power_control("i2c", "3v3")
    matrix_init(dev, int_pin=40, reset_pin=39)
    return (dev,)


@app.cell(hide_code=True)
def __(mo):
    mo.md(
        r"""
        # OTS 数据读取

        从 `xxx.toml` 中读取 OTS 相关配置，包括 slave-id (address) / chipid / register_list。

        下面代码中定义了简单的 test item —— `get_ots_chipid()`
        """
    )
    return


@app.cell
def __(dev, time, toml):
    with open("./mot6010_config.toml", "r", encoding="utf-8") as f:
        config = toml.load(f)

    ots_address = config["address"]
    chipid = config["chipid"]
    register_list = config["register_list"]


    def get_ots_chipid():
        """
        @brief get the chipid of OTS
        @return chipid
        """

        dev.i2c_write_read(slave_id=ots_address, write_list=[0x06, 0x97], read_length=0, port=0)

        time.sleep(5 / 1000)

        for reg in register_list:
            ret, _ = dev.i2c_write_read(slave_id=ots_address, write_list=reg, read_length=0, port=0)
            if ret is not True:
                print(f"write register 0x{reg[0]:02X},0x{reg[1]:02X} failed")
                return 0x0000

        ret, data = dev.i2c_write_read(slave_id=ots_address, write_list=[0x00], read_length=2, port=0)
        if ret is not True:
            return 0x0000

        chipid = data[0] << 8 | data[1]

        return chipid
    return chipid, config, f, get_ots_chipid, ots_address, register_list


@app.cell(hide_code=True)
def __(mo):
    mo.md(
        r"""
        # 测试配置
        1. 请在下面输入测试所需要的 **次数**、**起始点** 的X/Y值 以及 **终止点** 的 X/Y 值。
        2. 测试结束后会显示测试结果矩阵
        """
    )
    return


@app.cell(hide_code=True)
def __(mo):
    state = {
        "test_counts": 10000,
        "start_x": 0,
        "start_y": 0,
        "end_x": 7,
        "end_y": 7,
    }

    ui_array = mo.ui.array(
        [
            mo.ui.number(
                value=state["test_counts"],
                label="测试次数",
                step=1,
                on_change=lambda value: state.update(test_counts=value),
            ),
            mo.ui.number(
                value=state["start_x"],
                label="起始点 X 值",
                step=1,
                on_change=lambda value: state.update(start_x=value),
            ),
            mo.ui.number(
                value=state["start_y"],
                label="起始点 Y 值",
                step=1,
                on_change=lambda value: state.update(start_y=value),
            ),
            mo.ui.number(
                value=state["end_x"],
                label="终止点 X 值",
                step=1,
                on_change=lambda value: state.update(end_x=value),
            ),
            mo.ui.number(
                value=state["end_y"],
                label="终止点 Y 值",
                step=1,
                on_change=lambda value: state.update(end_y=value),
            ),
        ]
    )

    ui_array
    return state, ui_array


@app.cell
def __(np):
    test_data = np.zeros((8, 8), dtype=np.uint32)
    failed_data = np.zeros((8, 8), dtype=np.uint32)
    return failed_data, test_data


@app.cell
def __(
    dev,
    failed_data,
    get_ots_chipid,
    matrix_power_control,
    state,
    test_data,
    trange,
):
    for index in trange(test_data[0, 0], state["test_counts"], 1, desc="OTS Matrix Test"):
        for x in range(state["start_x"], state["end_x"] + 1):
            for y in range(state["start_y"], state["end_y"] + 1):
                # print(x, y)
                matrix_power_control(dev, x, y, True)
                data = get_ots_chipid()

                test_data[x, y] += 1

                if data != 0x585B:
                    print(f"chipid read failed (0x{data:04X}) in ({x, y})")
                    failed_data[x, y] += 1
    return data, index, x, y


@app.cell
def __(failed_data, matrix_display, state, test_data):
    matrix_display(failed_data, test_data, state["start_x"], state["start_y"], state["end_x"], state["end_y"])
    return


@app.cell
def __(dev, matrix_power_control):
    matrix_power_control(dev, 0, 3, True)
    return


@app.cell
def __(dev):
    dev.i2c_write_read(0x20, [0x02], 2)
    return


@app.cell
def __():
    bin(7)
    return


@app.cell
def __(dev, matrix_power_control):
    while True:
        matrix_power_control(dev, 0, 0, True)
        matrix_power_control(dev, 0, 1, True)
    return


@app.cell
def __(dev, matrix_power_control):
    matrix_power_control(dev, 0, 0, True)
    return


@app.cell
def __(dev):
    dev.i2c_write_read(0x20, [0x02], 2)
    return


@app.cell
def __(dev):
    bin(dev.i2c_write_read(0x18, [0x01], 1)[1][0])
    return


@app.cell
def __():
    return


if __name__ == "__main__":
    app.run()
