import marimo

__generated_with = "0.9.32"
app = marimo.App(app_title="Motor Contoller ")


@app.cell
def __(__file__):
    import os
    import sys

    import marimo as mo

    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

    import time

    from mxESP32Debugger.debugger import Dbg as MXDBG
    return MXDBG, mo, os, sys, time


@app.cell(hide_code=True)
def __(mo):
    mo.md(
        r"""
        # 创建一个 MXDBG 对象

        > 创建成功后可以看到下面提示所连接的 **COM 口**、**设备信息** 和 **软件信息**。
        """
    )
    return


@app.cell
def __(MXDBG):
    dev = MXDBG()
    return (dev,)


@app.cell(hide_code=True)
def __(mo):
    mo.md(
        r"""
        # 配置 PWM 基本参数

        1. 确定当前电机（如型号为 `FYDB504T`）的 **电机脉冲步长** （*Motor Step about Pulse*，简写为 $MSaP$），单位为 **脉冲每转**（*Pulse/Round*）。
            - 例如，当前的电机脉冲步长为 `10000 Pulse/Round`，即电机每收到 10000 个 PWM 方波时就会转 1圈。
        2. 确定所需要的 **电机转速** $MS$ ，单位是 **转每秒** （*Round/Second*）或 **转每分钟** （*Round/Miniute*）。
        4. `Freq` 即频率，即每秒要输出多少脉冲，单位 **脉冲每转** （*Pulse/Second*）。

        三者间的转换公式如下：
        $$Freq=MSaP*MS$$

        或者写成单位的形式：
        $$Pulse/Second=Pulse/Round * Round/Second$$
        """
    )
    return


@app.cell
def __():
    def get_freq(rpm: int) -> int:
        MSaP = 10_000
        return int(rpm / 60 * MSaP)


    # 例如，求算 10RPM 对应的 Freq
    rpm = 10
    freq = get_freq(rpm)
    return freq, get_freq, rpm


@app.cell
def __(dev):
    dev.pwm_config(pin=16, freq=16660, duty=0.5, channel=0, resolution_hz=80_000_000)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# 运行 PWM""")
    return


@app.cell
def __(dev):
    dev.pwm_run_stop(pwm_running_state=True, channel=0)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(
        r"""
        # 控制方向

        1. 将方向控制引脚设定在 40 pin，请连接好对应的引脚。
        2. 通过 `gpio_config()` 配置为输出模式 + 下拉。
        3. 然后通过 `gpio_write_read()` 来设置拉高或拉低。
        """
    )
    return


@app.cell
def __(dev, time):
    dev.gpio_config(pin=40, mode=0x02, pull_up=False, pull_down=True)
    time.sleep(2.5)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""## 反向转动""")
    return


@app.cell
def __(dev, time):
    dev.gpio_write_read(pin=40, level=1)
    time.sleep(2.5)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""## 正向转动""")
    return


@app.cell
def __(dev, time):
    dev.gpio_write_read(pin=40, level=0)
    time.sleep(5)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# 停止 PWM""")
    return


@app.cell
def __(dev):
    dev.pwm_run_stop(pwm_running_state=False, channel=0)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(
        r"""
        # 复位设备

        如果你想重置设备，不需要插拔USB端口，只需要发一句 `restart()` 即可。
        """
    )
    return


@app.cell
def __(dev):
    dev.restart()
    return


if __name__ == "__main__":
    app.run()
