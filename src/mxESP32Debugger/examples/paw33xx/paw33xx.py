import marimo

__generated_with = "0.9.32"
app = marimo.App(width="medium")


@app.cell
def __():
    import marimo as mo
    return (mo,)


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# Import Libries""")
    return


@app.cell
def __():
    from mxESP32Debugger.debugger import Dbg
    import toml
    import time
    from tqdm import trange
    return Dbg, time, toml, trange


@app.cell
def __(Dbg):
    dev = Dbg()
    dev.power_init()
    return (dev,)


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# Common Operation""")
    return


@app.cell
def __(dev):
    # 给 LED 通电 1.8V
    dev.expand_io_write_read("1v8_1", 1)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# PAW3395""")
    return


@app.cell
def __(toml):
    config = toml.load("paw3395.toml")
    return (config,)


@app.cell(hide_code=True)
def __(Dbg):
    def paw3395_init(config: dict, dev: Dbg):
        """
        Initialize the PAW3395 sensor
        """

        for reg in config["init"]["reg_list_1"]:
            ret, data = dev.spi_write_read(write_list=reg, read_length=0)

        ret, data = dev.spi_write_read(
            write_list=[0x6C, 0x00],
            read_length=2,
            critical_mode=True,
            justice=0x80,
            justice_index=1,
            timeout=60,
            examine_period=1,
        )

        if ret is False:
            for reg in config["init"]["reg_list_2"]:
                ret, data = dev.spi_write_read(write_list=reg, read_length=0)

        for reg in config["init"]["reg_list_3"]:
            ret, data = dev.spi_write_read(write_list=reg, read_length=0)


    def paw3395_switch_mode(config: dict, dev: Dbg, mode: int = 0):
        """
        Switch the PAW3395 sensor mode
        """

        modes = ["high_performance_mode", "low_power_mode", "office_mode", "corded_gaming_mode"]
        if mode not in range(len(modes)):
            raise ValueError(f"mode {mode} is not supported")

        reg_list = config["mode_reg_list"][modes[mode]]

        for reg in reg_list:
            ret, data = dev.spi_write_read(write_list=reg, read_length=0)
            assert ret, f"SPI write {reg} failed"

        if mode < 3:
            ret, data = dev.spi_write_read(write_list=[0x40, 0x00], read_length=2)
            assert ret, f"SPI read 0x40 failed"

            temp_data = data[1] & mode

            ret, data = dev.spi_write_read(write_list=[0x40, temp_data], read_length=0)
            assert ret, f"SPI write 0x40 failed"


    def paw3395_read_dx_dy(dev: Dbg):
        """
        Read the delta x and delta y from the PAW3395 sensor
        """

        reg_list = range(0x02, 0x07)
        reg_value = []

        for reg in reg_list:
            ret, data = dev.spi_write_read(write_list=[reg, 0x00], read_length=2)
            assert ret, f"SPI read {reg} failed"
            # print(ret, data)
            reg_value.append(data[1])

        motion, dx_l, dx_h, dy_l, dy_h = reg_value
        dx = (dx_h << 8) | dx_l
        dy = (dy_h << 8) | dy_l

        # 0x0000 -> 0
        # 0xFFFF -> -1
        # 0x7FFF -> 32767
        # 0x8000 -> -32768

        if dx & 0x8000:
            dx = dx - 0x10000
        if dy & 0x8000:
            dy = dy - 0x10000

        return motion, dx, dy
    return paw3395_init, paw3395_read_dx_dy, paw3395_switch_mode


@app.cell
def __(config, dev, paw3395_init, paw3395_switch_mode):
    paw3395_init(config, dev)
    paw3395_switch_mode(config, dev, 0)
    return


@app.cell
def __(config, dev):
    reset_pin = 39
    dev.gpio_config(pin=reset_pin, mode=0x02, pull_up=True, pull_down=False)
    dev.power_control("spi", config["electronic"]["vdd_voltage"])
    return (reset_pin,)


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""## Read dx/dy""")
    return


@app.cell
def __(dev, paw3395_read_dx_dy):
    while True:
        mot, dx, dy = paw3395_read_dx_dy(dev)
        if (dx != 0) or (dy != 0):
            print(mot, dx, dy)
    return dx, dy, mot


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""## Read Raw Image""")
    return


@app.cell(hide_code=True)
def __(Dbg, reset_pin, trange):
    def read_image(dev: Dbg):
        """
        Read the raw image data from the sensor
        """

        dev.gpio_write_read(reset_pin, 0)
        dev.gpio_write_read(reset_pin, 1)

        ret, _ = dev.spi_write_read([0x7F, 0x00], 0)
        assert ret, "SPI write_read failed"
        ret, _ = dev.spi_write_read([0x40, 0x80], 0)
        assert ret, "SPI write_read failed"

        # Continuously read reg 0x02 until OP_Mode1 and OP_Mode0 are both 0
        loop_counts_max = 100
        while True:
            ret, data = dev.spi_write_read([0x02, 0x00], 2)
            assert ret, "SPI write_read failed"
            if data[1] & 0x03 == 0x00:
                break
            loop_counts_max -= 1
            if loop_counts_max == 0:
                raise RuntimeError("Read image timeout")

        ret, _ = dev.spi_write_read([0x50, 0x01], 0)
        assert ret, "SPI write_read failed"
        ret, _ = dev.spi_write_read([0x55, 0x04], 0)
        assert ret, "SPI write_read failed"
        ret, _ = dev.spi_write_read([0x58, 0xFF], 0)
        assert ret, "SPI write_read failed"

        # Continuously read reg 0x59 until PG_FIRST and PG_VALID are both 1
        loop_counts_max = 1000
        while True:
            ret, data = dev.spi_write_read([0x59, 0x00], 2)
            assert ret, "SPI write_read failed"
            if ((data[1] & 0xC0) >> 6) == 0x03:
                break
            loop_counts_max -= 1
            if loop_counts_max == 0:
                print(data[1])
                raise RuntimeError("Read image timeout")

        ret, data = dev.spi_write_read([0x58, 0x00], 2)
        assert ret, "SPI write_read failed"

        image = []
        image.append(data[1])

        for i in trange(1295, desc="Image Reading"):
            # Continuously read reg 0x59 until PG_VALID is 1
            loop_counts_max = 100
            while True:
                ret, data = dev.spi_write_read([0x59, 0x00], 2)
                assert ret, "SPI write_read failed"
                if ((data[1] & 0x80) >> 7) == 0x01:
                    break
                loop_counts_max -= 1
                if loop_counts_max == 0:
                    raise RuntimeError("Read image timeout")

            ret, data = dev.spi_write_read([0x58, 0x00], 2)
            assert ret, "SPI write_read failed"

            image.append(data[1])

        ret, _ = dev.spi_write_read([0x40, 0x00], 0)
        assert ret, "SPI write_read failed"
        ret, _ = dev.spi_write_read([0x50, 0x00], 0)
        assert ret, "SPI write_read failed"
        ret, _ = dev.spi_write_read([0x55, 0x00], 0)
        assert ret, "SPI write_read failed"

        return image
    return (read_image,)


@app.cell
def __(dev, reset_pin):
    dev.gpio_write_read(reset_pin, 0)
    dev.gpio_write_read(reset_pin, 1)
    dev.spi_read_image(36,36)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# PAW3311""")
    return


@app.cell
def __(dev):
    # PAW3311 用的是3线半双工通信，需要执行下面代码进行配置
    dev.spi_config(
        freq=1_000_000,  # 1MHz
        miso_io_num=-1,  # the key point to config as a half duplex 3-wire mode
        cs_ena_pretrans=1,
        cs_ena_posttrans=1,
        device_interface_flags=(dev.spi_device["SPI_DEVICE_HALFDUPLEX"] | dev.spi_device["SPI_DEVICE_3WIRE"]),
    )
    return


@app.cell
def __(toml):
    config = toml.load("paw3311.toml")
    return (config,)


@app.cell(hide_code=True)
def __(Dbg):
    def paw3311_init(config: dict, dev: Dbg):
        """
        Initialize the PAW3311 sensor
        """

        for reg in config["init"]["reg_list_1"]:
            ret, _ = dev.spi_write_read(reg, 0)
            assert ret, f"SPI write {reg} failed"

        ret, data = dev.spi_write_read([0x46], 1)
        assert ret, f"SPI read 0x46 failed"
        R1 = data[0]

        for reg in config["init"]["reg_list_2"]:
            ret, _ = dev.spi_write_read(reg, 0)
            assert ret, f"SPI write {reg} failed"

        ret, data = dev.spi_write_read([0x46], 1)
        assert ret, f"SPI read 0x46 failed"
        R2 = data[0]

        temp_reg_list = config["init"]["reg_list_3"]
        temp_reg_list.append([0x6A, R1])
        temp_reg_list.append([0x6C, R2])

        print(f"R1: {R1}, R2: {R2}")

        for reg in temp_reg_list:
            ret, _ = dev.spi_write_read(reg, 0)
            assert ret, f"SPI write {reg} failed"

        for reg in config["init"]["reg_list_4"]:
            ret, _ = dev.spi_write_read(reg, 0)
            assert ret, f"SPI write {reg} failed"

        ret, data = dev.spi_write_read(
            write_list=[0x20],
            read_length=1,
            critical_mode=True,
            justice=0x0F,
            justice_index=0,
            timeout=155,
            examine_period=1,
        )
        assert ret, f"SPI read 0x20 failed, data read:0x{data[0]:02X}"

        for reg in config["init"]["reg_list_5"]:
            ret, _ = dev.spi_write_read(reg, 0)
            assert ret, f"SPI write {reg} failed"


    def paw3311_read_dx_dy(dev: Dbg):
        """
        Read the delta x and delta y from the PAW3395 sensor
        """

        reg_list = range(0x02, 0x07)
        reg_value = []

        for reg in reg_list:
            ret, data = dev.spi_write_read(write_list=[reg], read_length=1)
            assert ret, f"SPI read {reg} failed"
            # print(ret, data)
            reg_value.append(data[0])

        motion, dx_l, dx_h, dy_l, dy_h = reg_value
        dx = (dx_h << 8) | dx_l
        dy = (dy_h << 8) | dy_l

        # 0x0000 -> 0
        # 0xFFFF -> -1
        # 0x7FFF -> 32767
        # 0x8000 -> -32768

        if dx & 0x8000:
            dx = dx - 0x10000
        if dy & 0x8000:
            dy = dy - 0x10000

        return motion, dx, dy
    return paw3311_init, paw3311_read_dx_dy


@app.cell
def __(config, dev, paw3311_init):
    dev.power_control("spi", config["electronic"]["vdd_voltage"])
    paw3311_init(config, dev)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""## Read dx/dy""")
    return


@app.cell
def __(dev, paw3311_read_dx_dy):
    while True:
        mot, dx, dy = paw3311_read_dx_dy(dev)
        if dx != 0 or dy != 0:
            print(mot, dx, dy)
    return dx, dy, mot


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# Motor Control""")
    return


@app.cell
def __(dev):
    dev.pwm_config(pin=16, freq=200_000, duty=0.5, channel=0, resolution_hz=80_000_000)
    return


@app.cell
def __(dev):
    dev.pwm_run_stop(pwm_running_state=False, channel=0)
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# Restart Device""")
    return


@app.cell
def __(dev):
    dev.restart()
    return


@app.cell(hide_code=True)
def __(mo):
    mo.md(r"""# SPI Test""")
    return


@app.cell
def __(dev, trange):
    for i in trange(10000, desc="SPI test"):
        ret, data = dev.spi_write_read([0x00], 1)
        assert ret, "SPI write_read failed"
        assert data[0] == 0x50, f"SPI read data error: {data[0]}"
    return data, i, ret


@app.cell
def __(dev):
    ret, data = dev.spi_write_read([0x58, 0x00], 2)
    assert ret, "SPI write_read failed"
    print(data[1])
    return data, ret


@app.cell
def __():
    return


if __name__ == "__main__":
    app.run()
