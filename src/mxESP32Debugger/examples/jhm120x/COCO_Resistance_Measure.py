import marimo

__generated_with = "0.9.32"
app = marimo.App(width="full")


@app.cell
def __():
    import marimo as mo
    from mxESP32Debugger.debugger import Dbg

    import numpy as np
    import pandas as pd
    import time
    import ctypes

    from tqdm import trange
    import matplotlib.pyplot as plt
    return Dbg, ctypes, mo, np, pd, plt, time, trange


@app.cell
def __(Dbg):
    dev = Dbg()
    dev.power_init()
    dev.i2c_config(sda_pullup=False, scl_pullup=False, freq=400_000)
    dev.power_control("i2c", "3v3")

    assert "0x78" in dev.i2c_find_slave()[1], "No device found"
    coco_addr = 0x78
    return coco_addr, dev


@app.cell
def __(coco_addr, ctypes, dev, time):
    def coco_get_value():
        def check_status(status):
            power_indication = True if status & 0x40 else False
            busy_indication = True if status & 0x20 else False
            cmd_enable = True if status & 0x08 else False
            flag_memory_integrity = True if status & 0x04 else False

            # logger.debug(f'power_indication: {power_indication}')
            # logger.debug(f'busy_indication: {busy_indication}')
            # print(f'cmd_enable: {cmd_enable}')
            # print(f'flag_memory_integrity: {flag_memory_integrity}')

            return power_indication, busy_indication, cmd_enable, flag_memory_integrity

        ret, data = dev.i2c_write_read(coco_addr, [0xA3, 0xC7, 0x00], 0)
        assert ret == True, "Failed to write to COCO"
        # print(ret, data)

        while True:
            ret, data = dev.i2c_write_read(coco_addr, [], 1)
            time.sleep(0.001)

            _, busy_indication, _, _ = check_status(data[0])

            if busy_indication == False:
                break

        ret, data = dev.i2c_write_read(coco_addr, [], 6)
        assert ret == True, "Failed to read from COCO"
        # print(ret, data)

        bridge_data = data[1] << 16 | data[2] << 8 | data[3]
        if bridge_data & 0x800000:
            bridge_data |= 0xFF000000
            bridge_data = ctypes.c_int32(bridge_data).value
        # logger.info(f'bridge_data: {bridge_data}')

        temperature = data[4] << 8 | data[5]
        # logger.info(f'temperature: {temperature}')

        return bridge_data, temperature
    return (coco_get_value,)


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# Sample Data""")
    return


@app.cell(hide_code=True)
def __(mo, pd):
    df = pd.DataFrame(columns=["R1", "R2", "R3", "R4", "COCO", "bridge_raw_mean", "temp_raw_mean"])
    data_dict = {"r1": 2_000, "r2": 1_500, "r3": 2_000, "r4": 2_000, "coco": 0, "sample_rate": 30}

    ui_array = mo.ui.array(
        [
            mo.ui.number(
                start=0,
                stop=10_000,
                step=100,
                value=data_dict["r1"],
                label="R1",
                on_change=lambda value: data_dict.update(r1=value),
            ),
            mo.ui.number(
                start=0,
                stop=10_000,
                step=50,
                value=data_dict["r2"],
                label="R2",
                on_change=lambda value: data_dict.update(r2=value),
            ),
            mo.ui.number(
                start=0,
                stop=10_000,
                step=100,
                value=data_dict["r3"],
                label="R3",
                on_change=lambda value: data_dict.update(r3=value),
            ),
            mo.ui.number(
                start=0,
                stop=10_000,
                step=100,
                value=data_dict["r4"],
                label="R4",
                on_change=lambda value: data_dict.update(r4=value),
            ),
            mo.ui.number(
                start=0,
                stop=10_000,
                step=100,
                value=data_dict["coco"],
                label="COCO",
                on_change=lambda value: data_dict.update(coco=value),
            ),
            mo.ui.number(
                start=0,
                stop=500,
                step=1,
                value=data_dict["sample_rate"],
                label="SampleRate",
                on_change=lambda value: data_dict.update(sample_rate=value),
            ),
        ]
    )
    ui_array
    return data_dict, df, ui_array


@app.cell(hide_code=True)
def __(coco_get_value, data_dict, df, np, trange):
    raw_bridge_list = []
    raw_temp_list = []
    for i in trange(data_dict["sample_rate"]):
        bridge_raw, temp_raw = coco_get_value()
        raw_bridge_list.append(bridge_raw)
        raw_temp_list.append(temp_raw)

    df.loc[len(df)] = [
        data_dict["r1"],
        data_dict["r2"],
        data_dict["r3"],
        data_dict["r4"],
        data_dict["coco"],
        np.mean(raw_bridge_list),
        np.mean(raw_temp_list),
    ]

    df.to_csv("data.csv", index=False)
    df
    return bridge_raw, i, raw_bridge_list, raw_temp_list, temp_raw


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# Data Plot""")
    return


@app.cell
def __(np, pd, plt):
    df_ = pd.read_csv("data_2.csv")
    x = df_["bridge_raw_mean"]
    y = df_["COCO"]

    # 二阶拟合
    coefficients = np.polyfit(x, y, 2)  # 拟合出二阶多项式系数
    a, b, c = coefficients  # 提取系数
    print(f"拟合公式：y = {a:.4f}x² + {b:.4f}x + {c:.4f}")

    # 生成拟合曲线
    x_fit = np.linspace(x.min(), x.max(), 500)  # 生成平滑的 x 轴数据
    y_fit = np.polyval(coefficients, x_fit)  # 计算拟合曲线上的 y 值

    # 创建子图，1行2列
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))  # 设置图形大小

    # 左侧：原始数据和拟合曲线
    ax[0].plot(x, y, "o", label="Raw Data")
    ax[0].plot(x_fit, y_fit, "-", label=f"Fit: y = {a:.4e}x² + {b:.4e}x + {c:.4f}")
    ax[0].set_ylabel("COCO Resistance Value(Ohm)")
    ax[0].set_xlabel("Bridge Raw Mean")
    ax[0].set_title("COCO Resistance Value vs Bridge Raw Mean")
    ax[0].grid()
    ax[0].legend()

    # 右侧：电路图
    circuit = plt.imread("Circuit.png")
    ax[1].imshow(circuit)
    ax[1].axis("off")  # 隐藏坐标轴
    ax[1].set_title("Circuit")

    # 在电路图下方显示 R1 ~ R4 的值
    resistor_text = f"R1 = {df_["R1"].mean()/1000:.3f}kΩ, R2 = {df_["R2"].mean()/1000:.3f}kΩ, R3 = {df_["R3"].mean()/1000:.3f}kΩ, R4 = {df_["R4"].mean()/1000:.3f}kΩ"
    ax[1].text(0.5, -0.1, resistor_text, fontsize=12, ha="center", transform=ax[1].transAxes)

    # 显示图形
    plt.tight_layout()  # 自动调整子图布局
    plt.show()
    return (
        a,
        ax,
        b,
        c,
        circuit,
        coefficients,
        df_,
        fig,
        resistor_text,
        x,
        x_fit,
        y,
        y_fit,
    )


@app.cell
def __():
    return


if __name__ == "__main__":
    app.run()
