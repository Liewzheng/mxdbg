import marimo

__generated_with = "0.9.32"
app = marimo.App(width="medium")


@app.cell
def __():
    import marimo as mo
    return (mo,)


@app.cell
def __():
    from mxESP32Debugger.debugger import Dbg
    import toml
    import time
    return Dbg, time, toml


@app.cell
def __(Dbg, toml):
    config = toml.load("paw3395.toml")
    dev = Dbg()
    dev.power_init()
    dev.power_control("spi", config["electronic"]["vdd_voltage"])
    return config, dev


@app.cell(hide_code=True)
def __(Dbg):
    def paw3395_init(config: dict, dev: Dbg):
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


    def paw3395_read_dx_dy(dev: Dbg):
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
    return paw3395_init, paw3395_read_dx_dy


@app.cell
def __(config, dev, paw3395_init):
    paw3395_init(config, dev)
    return


@app.cell
def __(dev):
    dev.expand_io_write_read("1v8_1", 1)
    return


@app.cell(hide_code=True)
def __(Dbg):
    def paw33xx_switch_mode(config: dict, dev: Dbg, mode: int = 0):
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
    return (paw33xx_switch_mode,)


@app.cell
def __(config, dev, paw33xx_switch_mode):
    paw33xx_switch_mode(config, dev, 0)
    return


@app.cell
def __(dev, paw3395_read_dx_dy, time):
    while True:
        print(paw3395_read_dx_dy(dev))
        time.sleep(0.05)
    return


@app.cell
def __(dev):
    dev.pwm_config(pin=16, freq=200_000, duty=0.5, channel=0, resolution_hz=80_000_000)
    return


@app.cell
def __(dev):
    dev.pwm_run_stop(pwm_running_state=False, channel=0)
    return


@app.cell
def __():
    return


if __name__ == "__main__":
    app.run()
