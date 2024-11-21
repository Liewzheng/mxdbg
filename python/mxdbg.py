'''
* Copyright (c) 2024 MixoSense Technology Ltd <contact@mixosense.com>.
*
* All rights are reserved.
* Proprietary and confidential.
* Unauthorized copying of this file, via any medium is strictly prohibited.
* Any use is subject to an appropriate license granted by MixoSense Technology
* Ltd.
'''

try:

    from loguru import logger
except ImportError:
    print("Please install loguru package.Try 'pip install loguru'.")
    raise ImportError

import ctypes
from types import MappingProxyType
import time

try:
    from serial import Serial
    import serial.tools.list_ports
except ImportError:
    print("Please install pyserial package.Try 'pip install pyserial'.")
    raise ImportError


class MXDBG:

    def __init__(self, *args, **kwargs):
        self._client = None
        self._last_execute_time = None
        self.pwm_run_state = False

        self._crc_enable = True

        # task cmd
        __constants_task__ = {
            "TASK_IDLE": 0,  # heartbeat task
            "TASK_I2C_WRITE_READ": 1,
            "TASK_I2C_CONFIG": 2,
            "TASK_GPIO_WRITE_READ": 3,
            "TASK_GPIO_CONFIG": 4,
            "TASK_SPI_WRITE_READ": 5,
            "TASK_SPI_CONFIG": 6,
            "TASK_PWM_RUN_STOP": 7,
            "TASK_PWM_CONFIG": 8,
            "TASK_SPI_READ_IMAGE": 9,
            "TASK_USB_CONFIG": 240,
        }

        self.task_cmd = MappingProxyType(__constants_task__)

        __constants_gpio_mode__ = {
            "GPIO_MODE_DISABLE": 0,
            "GPIO_MODE_INPUT": 1,
            "GPIO_MODE_OUTPUT": 2,
            "GPIO_MODE_OUTPUT_OD": 6,
            "GPIO_MODE_INPUT_OUTPUT_OD": 7,
            "GPIO_MODE_INPUT_OUTPUT": 3,
        }

        self.gpio_mode = MappingProxyType(__constants_gpio_mode__)

        __constants_usb_info__ = {
            "USB_VID": 0x303A,
        }

        self.usb_info = MappingProxyType(__constants_usb_info__)

        __constants_spi_common_bus_flag__ = {
            "SPICOMMON_BUSFLAG_SLAVE": 0,  # Initialize I/O in slave mode
            "SPICOMMON_BUSFLAG_MASTER": 1 << 0,  # Initialize I/O in master mode
            "SPICOMMON_BUSFLAG_IOMUX_PINS": 1 << 1,  # Check using iomux pins.
            # Or indicates the pins are configured through the IO mux rather than GPIO matrix.
            "SPICOMMON_BUSFLAG_GPIO_PINS": 1 << 2,   # Force the signals to be routed through GPIO matrix.
            "SPICOMMON_BUSFLAG_SCLK": 1 << 3,  # Check existing of SCLK pin. Or indicates CLK line initialized.
            "SPICOMMON_BUSFLAG_MISO": 1 << 4,  # Check existing of MISO pin. Or indicates MISO line initialized.
            "SPICOMMON_BUSFLAG_MOSI": 1 << 5,  # Check existing of MOSI pin. Or indicates MOSI line initialized.
            # Check MOSI and MISO pins can output. Or indicates bus able to work under DIO mode.
            "SPICOMMON_BUSFLAG_DUAL": 1 << 6,
            # Check existing of WP and HD pins. Or indicates WP & HD pins initialized.
            "SPICOMMON_BUSFLAG_WPHD": 1 << 7,
            # Check existing of MOSI and MISO pins as output. Or indicates bus able to work under QIO mode.
            # Check existing of MOSI/MISO/WP/HD pins as output. Or indicates bus able to work under QIO mode.
            "SPICOMMON_BUSFLAG_QUAD": 1 << 6 | 1 << 7,
            # Check existing of IO4~IO7 pins. Or indicates IO4~IO7 pins initialized.
            "SPICOMMON_BUSFLAG_IO4_IO7": 1 << 8,
            # Check existing of MOSI/MISO/WP/HD/SPIIO4/SPIIO5/SPIIO6/SPIIO7 pins as output. Or indicates bus able to work under octal mode.
            "SPICOMMON_BUSFLAG_OCTAL": 1 << 6 | 1 << 7 | 1 << 8,
            "SPICOMMON_BUSFLAG_NATIVE_PINS": 1 << 1,  # Check using iomux pins.
        }

        self.spi_common_bus_flag = MappingProxyType(__constants_spi_common_bus_flag__)

        __constants_spi_device__ = {
            "SPI_DEVICE_NONE": 0,
            "SPI_DEVICE_TXBIT_LSBFIRST": 1 << 0,  # Transmit command/address/data LSB first instead of the default MSB first
            "SPI_DEVICE_RXBIT_LSBFIRST": 1 << 1,  # Receive data LSB first instead of the default MSB first
            "SPI_DEVICE_BIT_LSBFIRST": 1 << 0 | 1 << 1,  # Transmit and receive LSB first
            "SPI_DEVICE_3WIRE": 1 << 2,  # Use MOSI (=spid) for both sending and receiving data
            "SPI_DEVICE_POSITIVE_CS": 1 << 3,  # Make CS positive during a transaction instead of negative
            "SPI_DEVICE_HALFDUPLEX": 1 << 4,  # Transmit data before receiving it, instead of simultaneously
            "SPI_DEVICE_CLK_AS_CS": 1 << 5,  # Output clock on CS line if CS is active
            # There are timing issue when reading at high frequency (the frequency is related to whether iomux pins are used, valid time after slave sees the clock).
            "SPI_DEVICE_NO_DUMMY": 1 << 6,
            # In half-duplex mode, the driver automatically inserts dummy bits before reading phase to fix the timing issue. Set this flag to disable this feature.
            "SPI_DEVICE_DDRCLK": 1 << 7,
            # In full-duplex mode, however, the hardware cannot use dummy bits, so there is no way to prevent data being read from getting corrupted.
            "SPI_DEVICE_NO_RETURN_RESULT": 1 << 8,
        }

        self.spi_device = MappingProxyType(__constants_spi_device__)

        __constants_ret_code_of_each_cmd__ = {


            "TASK_SPI_WRITE_READ": {
                "RET_OK": {
                    "value": 0,
                    "description": "Success.",
                },
                "RET_ERROR_LENGTH": {
                    "value": -1,
                    "description": "Wrong write_len and read_len",
                },
                "RET_ERROR_READ_LENGTH": {
                    "value": -2,
                    "description": "Read length should not be larger than write length",
                },
                "RET_ERROR_MEMORY_ALLOCATION": {
                    "value": -3,
                    "description": "Memory allocation failed",
                },
            },

            "TASK_PWM_RUN_STOP": {
                "RET_OK": {
                    "value": 0,
                    "description": "Success.",
                },
                "RET_ERROR_RUNNING": {
                    "value": -1,
                    "description": "PWM is already running",
                },
                "RET_ERROR_STOPPED": {
                    "value": -2,
                    "description": "PWM is already stopped.",
                },
            }
        }

        self.ret_code = MappingProxyType(__constants_ret_code_of_each_cmd__)

        self._all_valid_pins = [-1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
                                14, 15, 16, 17, 18, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42]
        self._pwm_used_pins = [16, 17, 18]
        self._i2c_used_pins = [10, 11]
        self._spi_used_pins = [12, 13, 14, 15]
        self._gpio_used_pins = set()

        self._gpio_valid_pins = None
        self._i2c_valid_pins = None
        self._spi_valid_pins = None
        self._pwm_valid_pins = None
        
        self.pca9557pw_addr = 0x18
        
        self._expand_io_init_status = False
        self._power_init_status = False
        self._expand_io_mode_bitmask = 0x00

        self.connect(**kwargs)        

    @property
    def pwm_valid_pins(self):
        if self._pwm_valid_pins is None:
            self._pwm_valid_pins = list(set(self._all_valid_pins) - set(self._gpio_used_pins) -
                                        set(self._spi_used_pins) - set(self._i2c_used_pins))
        return self._pwm_valid_pins

    @property
    def i2c_valid_pins(self):
        if self._i2c_valid_pins is None:
            self._i2c_valid_pins = list(set(self._all_valid_pins) - set(self._pwm_used_pins) -
                                        set(self._spi_used_pins) - set(self._gpio_used_pins))
        return self._i2c_valid_pins

    @property
    def spi_valid_pins(self):
        if self._spi_valid_pins is None:
            self._spi_valid_pins = list(set(self._all_valid_pins) - set(self._pwm_used_pins) -
                                        set(self._i2c_used_pins) - set(self._gpio_used_pins))
        return self._spi_valid_pins

    @property
    def gpio_valid_pins(self):
        if self._gpio_valid_pins is None:
            self._gpio_valid_pins = list(set(self._all_valid_pins) - set(self._pwm_used_pins) -
                                         set(self._spi_used_pins) - set(self._i2c_used_pins))
        return self._gpio_valid_pins

    def mark_pwm_used(self, pin):

        if pin == -1:
            logger.warning("PWM is not supported to set pin to -1.")
            return

        if pin in self.pwm_valid_pins:
            self._pwm_used_pins.append(pin)
            self._pwm_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not a valid PWM pin or already used by other module.')

    def mark_i2c_used(self, pin):

        if pin == -1:
            logger.warning("I2C is not supported to set pin to -1.")
            return

        if pin in self.i2c_valid_pins:
            self._i2c_used_pins.append(pin)
            self._i2c_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not a valid I2C pin or already used by other module.')

    def mark_spi_used(self, pin):

        if pin == -1:
            return

        if pin in self.spi_valid_pins:
            self._spi_used_pins.append(pin)
            self._spi_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not a valid SPI pin or already used by other module.')

    def mark_gpio_used(self, pin):

        if pin == -1:
            logger.warning("GPIO is not supported to set pin to -1.")
            return

        if pin in self.gpio_valid_pins:
            self._gpio_used_pins.add(pin)
            self._gpio_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not a valid GPIO pin or already used by other module.')

    def mark_pin_free(self, pin):
        '''
        check all the used pins if the pin was in it, then remove it.
        '''

        if pin == -1:
            return

        if pin in self._gpio_used_pins:
            self._gpio_used_pins.remove(pin)
            self._gpio_valid_pins = None
        elif pin in self._spi_used_pins:
            self._spi_used_pins.remove(pin)
            self._spi_valid_pins = None
        elif pin in self._i2c_used_pins:
            self._i2c_used_pins.remove(pin)
            self._i2c_valid_pins = None
        elif pin in self._pwm_used_pins:
            self._pwm_used_pins.remove(pin)
            self._pwm_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not used by any module. Or it is not a valid pin.')

    def connect(self, **kwargs):

        _port = None
        retry_times = 10
        while True:
            for port in sorted(serial.tools.list_ports.comports()):

                if port.vid is None or port.pid is None:
                    continue
                if (self.usb_info["USB_VID"] == port.vid):
                    _port = port.name
                    break

            if _port is not None:
                break
            else:
                retry_times -= 1
                time.sleep(1)

            if retry_times == 0:
                raise ValueError("No device found.")

        try:
            # use serial port to connect
            self._client = Serial(_port, 115200, timeout=5, write_timeout=1, **kwargs)
            logger.info("Using serial port to connect. Port: {}".format(_port))

        except Exception as e:
            raise ValueError(f"Failed to connect: {e}")

    def disconnect(self):
        self._client.close()

    def read(self, timeout=2):

        data = bytearray()
        start_time = time.time()

        while True:

            # 检查超时
            if time.time() - start_time > timeout:
                if len(data) == 0:
                    raise TimeoutError("Data read timeout. No data received.")
                else:
                    raise TimeoutError("Data read timeout. Data received: {}".format(data))

            if self._client.in_waiting > 0:

                temp_data = self._client.read(self._client.in_waiting)
                data += temp_data

            if len(data) >= 5:
                if self.check_crc(data) and (data[:5] == bytearray("mxdbg", "utf-8")):
                    break

            time.sleep(0.01)

        return data

    def write(self, data):
        self._client.write(data)

    def task_execute(self, cmd, data: list):

        write_data = self.data_pack(cmd, data)

        # self._client.flushInput()
        # self._client.flushOutput()

        # logger.debug("Write Data: {}".format(["{:02X} ".format(d) for d in list(write_data)]))

        try:
            self.write(write_data)
            self._last_execute_time = time.time()
        except Exception as e:
            raise ValueError(f"Failed to write data: {e}")

        read_data = self.read()

        ret, temp_data = self.data_unpack(cmd, read_data)

        return ret, temp_data

    def data_decompose(self, data: int, bytes_num: int = 4) -> tuple:
        return [(data >> (8 * (bytes_num - 1 - i))) & 0xFF for i in range(bytes_num)]

    def hexdump(self, data: bytearray, base_address=0x3fc9900c):
        logger.debug('hex:')

        # 按行分组数据，每行16字节
        for i in range(0, len(data), 16):

            # 显示16字节的 hex 值，每8字节后加一个 '\t'
            line_data = data[i:i+16]
            for j, byte in enumerate(line_data):
                print("{:02X} ".format(byte), end='')
                if (j + 1) % 8 == 0:
                    print('\t', end='')

            # 计算并显示 ASCII 表示，非可见字符替换为 '.'
            ascii_repr = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in line_data)
            print("|{}|".format(ascii_repr))

    def calculate_crc(self, data: bytearray) -> bytearray:
        '''
        @brief: Calculate CRC16 for the given data.
        @param data: Data to calculate CRC16.
        @return: CRC16 value.
        '''

        crc = 0xFFFF

        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ (0xA001 if (crc & 1) else 0)

        high, low = ((crc >> 8) & 0xFF), (crc & 0xFF)

        return bytearray([high, low])

    def check_crc(self, data: bytearray) -> bool:
        '''
        @brief: Check CRC16 for the given data.
        @param data: Data to check CRC16.
        @return: True if CRC16 is correct, False otherwise.
        '''

        if (self._crc_enable == False):
            return True

        high, low = self.calculate_crc(data[:-2])

        # logger.debug(f"CRC Check: {high}, {low}, {data[-2]}, {data[-1]}")

        return True if ((high == data[-2]) and (low == data[-1])) else False

    def data_pack(self, cmd: int, data: list) -> bool:
        '''
        @brief: Package data with header and CRC16.
        @param data: Data to package.
        '''

        # data sent format
        # | 5 bytes | 1 byte | 1 byte | 1 byte | n bytes | 1 byte | 2 bytes |
        # |---------|--------|--------|--------|---------|--------|---------|
        # | mxdbg   | :      | cmd    | :      | data    | :      | crc     |

        temp_data = bytearray("mxdbg:", "utf-8")
        temp_data += bytearray([cmd])
        temp_data += bytearray(":", "utf-8")
        temp_data += bytearray(data)
        temp_data += bytearray(":", "utf-8")

        high, low = self.calculate_crc(temp_data)
        temp_data += bytearray([high, low])

        return temp_data

    def data_unpack(self, cmd, data: bytearray) -> tuple:
        '''
        @brief: Unpackage data.
        @param data: Data to unpackage.
        @return: cmd and data.
        '''

        # data received format
        # | 5 bytes | 1 byte | 1 byte | 1 byte | 4 bytes | 1 byte | n bytes | 1 byte | 2 bytes |
        # |---------|--------|--------|--------|---------|--------|---------|--------|---------|
        # | mxdbg   | :      | cmd    | :      | ret     | :      | data    | :      | crc     |

        # logger.debug("Data: {}".format(["{:02X}".format(d) for d in list(data)]))

        # check crc first
        temp_data = data
        assert self.check_crc(temp_data), "CRC check failed."

        # check header
        header = data[:5]
        assert header == bytearray("mxdbg", "utf-8"), "Invalid header."

        # check cmd
        cmd_ = data[6]
        assert cmd_ == cmd, "Invalid cmd."

        ret = list(data[8:12])[0] << 24 | list(data[8:12])[1] << 16 | list(data[8:12])[2] << 8 | list(data[8:12])[3]
        ret = ctypes.c_int32(ret).value

        if len(data) > 15:  # data exists
            temp_data = data[13:-3]
        else:
            temp_data = None

        return ret, temp_data

    def check_ret_code(self, cmd, ret):

        # check ret code
        if ret == 0:
            return True

        else:
            # 匹配 self.ret_code 中的返回值，并打印错误信息
            for key, value in self.ret_code.items():
                if self.task_cmd[key] == cmd:
                    for k, v in value.items():
                        if v["value"] == ret:
                            logger.error(f"{v['description']}")
                            return False

    def i2c_find_slave(self, port: int = 0):
        
        '''
        @brief: Find I2C slave devices.
        @return: List of I2C slave devices.
        '''

        found_device_list = []
        for slave_id in range(0x04, 0x7F):
            try:
                ret, data = self.i2c_write_read(slave_id, [0x00], 0, port=port)
                if ret == True:
                    found_device_list.append(slave_id)
            except:
                continue

        if found_device_list != []:
            found_device_list = ['0x{:02X}'.format(slave_id) for slave_id in found_device_list]
            return True, found_device_list
        else:
            return False, None

    def i2c_write_read(self,
                       slave_id: int,
                       write_list: list,
                       read_length: int,
                       port: int = 0,
                       slave_id_10_bit: bool = False):
        '''
        @brief: Write and read data from I2C slave device. The default I2C pin is SDA: 12, SCL: 11.
        @param slave_id: I2C slave device address.
        @param write_list: Data to write to I2C slave device. String, list or bytearray type.
        @param read_length: Length of data to read from I2C slave device.
        @param port: I2C port number. Default is 0 or 1.
        @param slave_id_10_bit: True if 10-bit slave address, False otherwise.
        @return: Data read from I2C slave device.
        '''

        assert port in [0, 1], "Invalid port number."
        assert slave_id >= 0x04 and slave_id <= 0x07FF, "Invalid slave id."

        # 构造要发送的数据包
        slave_id = slave_id if not slave_id_10_bit else slave_id & 0x07FF
        wirte_len = len(write_list)
        read_len = read_length

        '''
        * value format received:
        * | 5 bytes | 1 byte | 1 byte | 1 byte | 1 byte   | 2 bytes  | 4 bytes      | 4 bytes     | n bytes         | 1 byte | 2 bytes |
        * |---------|--------|--------|--------|----------|----------|--------------|-------------|-----------------|--------|---------|
        * | mxdbg   | :      | cmd    | :      | i2c_port | slave_id | write_length | read_length | write_data_list | :      | crc     |
        '''

        i2c_data_temp = [port,
                         (slave_id & 0xFF00) >> 8, slave_id & 0x00FF,
                         (wirte_len & 0xFF000000) >> 24, (wirte_len & 0x00FF0000) >> 16,
                         (wirte_len & 0x0000FF00) >> 8, wirte_len & 0x000000FF,
                         (read_len & 0xFF000000) >> 24, (read_len & 0x00FF0000) >> 16,
                         (read_len & 0x0000FF00) >> 8, read_len & 0x000000FF]
        i2c_data_temp += write_list

        # 执行任务并读取返回的数据
        ret, data = self.task_execute(self.task_cmd["TASK_I2C_WRITE_READ"], i2c_data_temp)

        data = list(data) if data is not None else None

        if ret != 0:
            # logger.error(f"Error: {ret}")
            return False, None
        else:
            return True, data

    def i2c_config(self,
                   port: int = 0, freq: int = 400000,
                   sda_pin: int = 10, scl_pin: int = 11,
                   sda_pullup: bool = True, scl_pullup: bool = True):
        '''
        @brief: Configure I2C port.
        @param port: I2C port number.
        @param freq: I2C frequency.
        @param sda_pin: SDA pin number.
        @param scl_pin: SCL pin number.
        @param sda_pullup: SDA pull up resistor. True to enable, False to disable.
        @param scl_pullup: SCL pull up resistor. True to enable, False to disable.
        @return: Return True if success, False otherwise.
        '''

        if port not in [0, 1]:
            logger.error("Invalid port number.")
            return False

        self._i2c_valid_pins = None

        if (sda_pin not in self.i2c_valid_pins) or (scl_pin not in self.i2c_valid_pins):
            logger.error("Invalid pin number.")
            return False, None
        elif sda_pin in self._i2c_used_pins and scl_pin in self._i2c_used_pins:
            pass

        if sda_pin == scl_pin:
            logger.error("SDA and SCL pins should be different.")
            return False, None

        port = ctypes.c_uint8(port).value
        freq = ctypes.c_uint32(freq).value
        sda_pin = ctypes.c_uint8(sda_pin).value
        scl_pin = ctypes.c_uint8(scl_pin).value
        sda_pullup = 0xF if sda_pullup else 0x0
        scl_pullup = 0xF if scl_pullup else 0x0

        i2c_data_temp = [port,
                         (freq & 0xFF000000) >> 24, (freq & 0x00FF0000) >> 16,
                         (freq & 0x0000FF00) >> 8, freq & 0x000000FF,
                         sda_pin, scl_pin,
                         (sda_pullup << 4 | scl_pullup)
                         ]

        ret, data = self.task_execute(self.task_cmd["TASK_I2C_CONFIG"], i2c_data_temp)

        if ret != 0:
            logger.error(f"Error: {ret}")
            return False, None
        else:
            for pin in self._i2c_used_pins:
                self.mark_pin_free(pin)
            for pin in [sda_pin, scl_pin]:
                self.mark_i2c_used(pin)
            return True, None

    def gpio_write_read(self, pin: int, level: int = None):
        '''
        @brief: Write or read GPIO pin. if level is -1, read mode, otherwise write mode.
        @param pin: GPIO pin number.
        @param level: GPIO level. 0: Low, 1: High. -1 in default.
        @return: Return True and data if success, False otherwise.
        '''

        if pin not in self.gpio_valid_pins:
            logger.error("Invalid pin number.")
            return False, None

        pin = ctypes.c_uint8(pin).value

        if level is None:  # read mode
            operation = 1
            gpio_data_temp = [operation, pin, 0]
        else:             # write mode
            operation = 0
            gpio_data_temp = [operation, pin, ((ctypes.c_uint8(level).value) & 0x01)]

        ret, data = self.task_execute(self.task_cmd["TASK_GPIO_WRITE_READ"], gpio_data_temp)

        if ret == 0:
            return True, None if (operation == 0) else data[0]
        else:
            logger.error(f"Error: {ret}")
            return False, None

    def gpio_config(self, pin: int, mode: int, pull_up: int, pull_down: int) -> bool:
        '''
        @brief: Configure GPIO pin.
        @param pin: GPIO pin number.
        @param mode: GPIO mode. 0x00: Disable, 0x01: Input, 0x02: Output, 0x03: Input/Output, 0x06: Output Open Drain, 0x07: Input/Output Open Drain
        @param pull_up: Pull up resistor. 0x00: Disable, 0x01: Enable.
        @param pull_down: Pull down resistor. 0x00: Disable, 0x01: Enable.
        @return: Return value. 0: Success, other: Error.
        '''

        self._gpio_valid_pins = None

        if pin not in self.gpio_valid_pins:
            logger.error("Invalid pin number.")
            return False, None

        pin = ctypes.c_uint8(pin).value
        mode = ctypes.c_uint8(mode).value
        pull_up = ctypes.c_uint8(pull_up).value
        pull_down = ctypes.c_uint8(pull_down).value

        gpio_data_temp = [pin, mode, pull_up, pull_down]
        ret, data = self.task_execute(self.task_cmd["TASK_GPIO_CONFIG"], gpio_data_temp)

        if ret != 0:
            logger.error(f"Error: {ret}")
            return False
        else:
            self.mark_gpio_used(pin)
            return True

    def spi_write_read(self, write_list: list, read_length: int):
        '''
        @brief: Write and read data from SPI slave device. The default SPI pins are MISO: 12, MOSI: 13, SCLK: 14, CS: 15.
        @param write_list: Data to write to SPI slave device. List type.
        @param read_length: Length of data to read from SPI slave device.
        @return: Data read from SPI slave device.
        '''

        write_len = ctypes.c_uint32(len(write_list)).value
        read_len = ctypes.c_uint32(read_length).value

        if (write_len == 0) and (read_len == 0):
            logger.error("Write length and read length should not be zero.")
            return False, None
        else:
            if read_len > write_len:
                logger.error("Read length should not be larger than write length.")
                return False, None

        spi_data_temp = [(write_len & 0xFF000000) >> 24, (write_len & 0x00FF0000) >> 16,
                         (write_len & 0x0000FF00) >> 8, (write_len & 0x000000FF) >> 0,
                         (read_len & 0xFF000000) >> 24, (read_len & 0x00FF0000) >> 16,
                         (read_len & 0x0000FF00) >> 8, (read_len & 0x000000FF) >> 0]
        spi_data_temp += write_list

        ret, data = self.task_execute(self.task_cmd["TASK_SPI_WRITE_READ"], spi_data_temp)

        self.check_ret_code(self.task_cmd["TASK_SPI_WRITE_READ"], ret)

        if ret == 0:
            if read_len == 0:
                return True, None
            else:
                return True, list(data)
        else:
            return False, None

    def spi_config(self,

                   miso_io_num: int = 12, mosi_io_num: int = 13,
                   sclk_io_num: int = 14, cs_io_num: int = 15, common_bus_flags: int = 0,
                   freq: int = 1000000, mode: int = 3,

                   quadwp_io_num: int = -1, quadhd_io_num: int = -1,
                   data4_io_num: int = -1, data5_io_num: int = -1,
                   data6_io_num: int = -1, data7_io_num: int = -1,

                   max_transfer_sz: int = 4096,
                   isr_cpu_id: int = 0,
                   intr_flags: int = 0,

                   command_bits: int = 0, address_bits: int = 0, dummy_bits: int = 0,
                   duty_cycle_pos: int = 0, cs_ena_pretrans: int = 0, cs_ena_posttrans: int = 0,
                   input_delay_ns: int = 0,
                   device_interface_flags: int = 0,
                   queue_size: int = 7,
                   ):
        '''
        @brief: Configure SPI.
        @param miso_io_num: MISO pin number. Default is 12. Set to -1 if not used.
        @param mosi_io_num: MOSI pin number. Default is 13. 
        @param sclk_io_num: SCLK pin number. Default is 14.
        @param cs_io_num: CS pin number. Default is 15. Set to -1 if not used.
        @param quadwp_io_num: QUADWP pin number. Default is -1.
        @param quadhd_io_num: QUADHD pin number. Default is -1.
        @param data4_io_num: DATA4 pin number. Default is -1.
        @param data5_io_num: DATA5 pin number. Default is -1.
        @param data6_io_num: DATA6 pin number. Default is -1.
        @param data7_io_num: DATA7 pin number. Default is -1.
        @param max_transfer_sz: Maximum transfer size. Default is 4096.
        @param common_bus_flags: Common bus flags. Default is 0.
        @param isr_cpu_id: ISR CPU ID. Default is 0.
        @param intr_flags: Interrupt flags. Default is 0.
        @param command_bits: Command bits. Default is 0.
        @param address_bits: Address bits. Default is 0.
        @param dummy_bits: Dummy bits. Default is 0.
        @param mode: SPI mode. Default is 3. Available values are 0, 1, 2, 3.
        @param duty_cycle_pos: Duty cycle position. Default is 0.
        @param cs_ena_pretrans: CS enable pretrans. Default is 0.
        @param cs_ena_posttrans: CS enable posttrans. Default is 0.
        @param freq: SPI frequency. Default is 1000000.
        @param input_delay_ns: Input delay in ns. Default is 0.
        @param device_interface_flags: Device interface flags. Default is 0. Set (SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE) if 3-wired half duplex mode.
        @param queue_size: Queue size. Default is 7.
        @return: Return True if success, False otherwise.
        '''

        for pin in [mosi_io_num, miso_io_num, sclk_io_num, cs_io_num, quadhd_io_num, quadwp_io_num,
                    data4_io_num, data5_io_num, data6_io_num, data7_io_num]:
            if pin != -1 and ((pin not in self.spi_valid_pins) and (pin not in self._spi_used_pins)):
                logger.error(f"Invalid pin number: {pin}")
                return False, None

        assert mode in [0, 1, 2, 3], "Invalid mode."
        assert common_bus_flags in self.spi_common_bus_flag.values(), "Invalid common bus flags."
        assert isr_cpu_id in [0, 1, 2], "Invalid ISR CPU ID."

        spi_config_data_temp = []
        spi_config_data_temp.extend(self.data_decompose(mosi_io_num))
        spi_config_data_temp.extend(self.data_decompose(miso_io_num))
        spi_config_data_temp.extend(self.data_decompose(sclk_io_num))
        spi_config_data_temp.extend(self.data_decompose(quadwp_io_num))
        spi_config_data_temp.extend(self.data_decompose(quadhd_io_num))
        spi_config_data_temp.extend(self.data_decompose(data4_io_num))
        spi_config_data_temp.extend(self.data_decompose(data5_io_num))
        spi_config_data_temp.extend(self.data_decompose(data6_io_num))
        spi_config_data_temp.extend(self.data_decompose(data7_io_num))
        spi_config_data_temp.extend(self.data_decompose(max_transfer_sz, 2))
        spi_config_data_temp.extend(self.data_decompose(common_bus_flags))
        spi_config_data_temp.extend(self.data_decompose(isr_cpu_id, 1))
        spi_config_data_temp.extend(self.data_decompose(intr_flags))
        spi_config_data_temp.extend(self.data_decompose(command_bits, 1))
        spi_config_data_temp.extend(self.data_decompose(address_bits, 1))
        spi_config_data_temp.extend(self.data_decompose(dummy_bits, 1))
        spi_config_data_temp.extend(self.data_decompose(mode, 1))
        spi_config_data_temp.extend(self.data_decompose(duty_cycle_pos, 2))
        spi_config_data_temp.extend(self.data_decompose(cs_ena_pretrans, 2))
        spi_config_data_temp.extend(self.data_decompose(cs_ena_posttrans, 1))
        spi_config_data_temp.extend(self.data_decompose(freq))
        spi_config_data_temp.extend(self.data_decompose(input_delay_ns))
        spi_config_data_temp.extend(self.data_decompose(cs_io_num))
        spi_config_data_temp.extend(self.data_decompose(device_interface_flags))
        spi_config_data_temp.extend(self.data_decompose(queue_size))

        self.hexdump(spi_config_data_temp)

        ret, data = self.task_execute(self.task_cmd["TASK_SPI_CONFIG"], spi_config_data_temp)
        self.check_ret_code(self.task_cmd["TASK_SPI_CONFIG"], ret)

        if ret == 0:
            for pin in self._spi_used_pins:
                self.mark_pin_free(pin)
            for pin in [mosi_io_num, miso_io_num, sclk_io_num, cs_io_num, quadhd_io_num, quadwp_io_num,
                        data4_io_num, data5_io_num, data6_io_num, data7_io_num]:
                self.mark_spi_used(pin)
            return True, None
        else:
            return False, None

    def spi_read_image(self):
        '''
        @brief: Read image data from SPI slave device.
        @return: Image data.
        '''

        ret, data = self.task_execute(self.task_cmd["TASK_SPI_READ_IMAGE"], [])

        if ret == 0:
            return True, data
        else:
            return False, None

    def pwm_run_stop(self, pwm_running_state: bool, channel: int = 0):
        '''
        @brief: Run or stop PWM. It will generate a PWM signal with 10KHz frequency and 25% duty cycle in default.
        @param pwm_running_state: True to run PWM, False to stop PWM.
        @param channel: PWM channel. Default is 0. Available channels are 0, 1, 2.
        @return: Return True if success, False otherwise.
        '''

        if channel not in [0, 1, 2]:
            logger.error("Invalid channel number. Available channels are 0, 1, 2.")
            return False

        channel = ctypes.c_uint8(channel).value
        run_state = ctypes.c_uint8(pwm_running_state).value
        ret, data = self.task_execute(self.task_cmd["TASK_PWM_RUN_STOP"], [channel, run_state])
        
        if ret != 0:
            logger.error(f"Error: {ret}")
            return False

        self.check_ret_code(self.task_cmd["TASK_PWM_RUN_STOP"], ret)

        if pwm_running_state:
            self.pwm_run_state = True
        else:
            self.pwm_run_state = False

        return self.pwm_run_state, ret

    def pwm_config(self, pin: int = 16, freq: int = 10000, duty: float = 0.5, channel: int = 0):
        '''
        @brief: Configure PWM.
        @param pin: PWM pin number. pin of Channel 0: 16 (in default); pin of Channel 1: 17 (in default); pin of Channel 2: 18 (in default).
        @param freq: PWM frequency. Unit: Hz; Default is 10000 (10KHz). Frequency should be far less than timer resolution.
        @param duty: PWM duty cycle. Default is 0.5 (50%). Duty cycle should be in the range of 0.0 (0%) to 1.0 (100%).
        @param channel: PWM channel. Default is 0. Available channels are 0, 1, 2.
        @return: Return True if success, False otherwise.
        '''

        self._pwm_valid_pins = None

        if pin not in self.pwm_valid_pins:
            logger.error("Invalid pin number.")
            return False, None

        if duty < 0 or duty > 1:
            logger.error("Invalid duty cycle. Duty cycle should be in the range of 0.0 to 1.0")
            return False, None

        timer_resolution = 80000000 # 80MHz

        if freq > timer_resolution:
            logger.error("Invalid frequency. Frequency should be less than timer resolution.")
            return False, None

        resolution_hz = ctypes.c_uint32(timer_resolution).value
        period_ticks = ctypes.c_uint32(resolution_hz // freq).value
        duty_ticks = int(period_ticks * duty)

        pin = ctypes.c_uint8(pin).value
        channel = ctypes.c_uint8(channel).value

        logger.debug(f"Period Ticks: {period_ticks}, Duty Ticks: {duty_ticks}")

        pwm_data_temp = [(channel & 0xFF),
                         (pin & 0xFF),

                         (period_ticks & 0xFF000000) >> 24, (period_ticks & 0x00FF0000) >> 16,
                         (period_ticks & 0x0000FF00) >> 8, period_ticks & 0x000000FF,

                         (duty_ticks & 0xFF000000) >> 24, (duty_ticks & 0x00FF0000) >> 16,
                         (duty_ticks & 0x0000FF00) >> 8, duty_ticks & 0x000000FF,
                         ]

        ret, _ = self.task_execute(self.task_cmd["TASK_PWM_CONFIG"], pwm_data_temp)

        self.check_ret_code(self.task_cmd["TASK_PWM_CONFIG"], ret)

        if ret == 0:
            self._pwm_used_pins = [pin]

        if self.pwm_run_state:
            self.pwm_run_stop(True, channel)

        for pin in self._pwm_used_pins:
            self.mark_pin_free(pin)
        self.mark_pwm_used(pin)

        return True, None

    def usb_config(self, crc_enable: bool = False):
        '''
        @brief: Configure USB.
        @param crc_enable: True to enable CRC, False to disable CRC.
        @return: Return True if success, False otherwise.
        '''

        crc_enable = ctypes.c_uint8(crc_enable).value
        self._crc_enable = crc_enable

        ret, data = self.task_execute(self.task_cmd["TASK_USB_CONFIG"], [crc_enable])

        if ret == 0:
            return True, None
        else:
            return False, None
        
    def power_init(self):
        
        '''
        @brief: Initialize power.
        @return: Return True if success, False otherwise.
        '''

        if not self._expand_io_init_status:
            self.expand_io_init()
            
        if not self._power_init_status:
            
            ret, data = self.i2c_write_read(self.pca9557pw_addr, [0x03], 1, port=1)
            if ret is not True:
                logger.error("PCA9557PW read failed.")
                raise ValueError("PCA9557PW read failed.")
            
            bit_mask = data[0]

            bit_mask &= 0xE1 # 1110 0001
            
            ret, data = self.i2c_write_read(self.pca9557pw_addr, [0x03, bit_mask], 0, port=1)
            if ret is not True:
                logger.error("PCA9557PW write failed.")
                raise ValueError("PCA9557PW write failed.")
            
            self._expand_io_mode_bitmask = bit_mask
            self._power_init_status = True
        else:
            logger.info("Power already initialized.")
        
        return True

    def power_control(self, communication_type:str="SPI", power_type:str='1V8'):
        
        '''
        @brief: Control power for communication type.
        @param communication_type: Communication type. SPI or I2C.
        @param power_type: Power type. 1V8, 1V2 or 0V.
        @return: Return True if success, False otherwise.
        '''
        
        communication_type = communication_type.upper()
        power_type = power_type.upper()
        
        if communication_type not in ["SPI", "I2C"]:
            logger.error("Invalid communication type. Only SPI and I2C are supported.")
            return False
        
        if power_type not in ["1V8", "1V2", "0V"]:
            logger.error("Invalid power type. Only 1V8, 1V2 and 0V are supported.")
            return False
        
        ret, data = self.i2c_write_read(self.pca9557pw_addr, [0x01], 1, port=1)
        if ret is not True:
            logger.error("PCA9557PW read failed.")
            raise ValueError("PCA9557PW read failed.")
        
        bit_mask = data[0]
        
        if communication_type == 'SPI' and power_type == '1V8': # IO3, bit 3
            bit_mask |= 0x08 # 0000 1000
            bit_mask &= 0xFD # 1111 1101
        elif communication_type == 'SPI' and power_type == '1V2': # IO1, bit 1
            bit_mask |= 0x02 # 0000 0010
            bit_mask &= 0xF7 # 1111 0111
        elif communication_type == 'SPI' and power_type == '0V': # set IO3 and IO1 to 0
            bit_mask &= 0xF5 # 1111 0101
        elif communication_type == 'I2C' and power_type == '1V8': # IO4, bit 4
            bit_mask |= 0x10 # 0001 0000
            bit_mask &= 0xFB # 1111 1011
        elif communication_type == 'I2C' and power_type == '1V2': # IO2, bit 2
            bit_mask |= 0x04 # 0000 0100
            bit_mask &= 0xEF # 1110 1111
        elif communication_type == 'I2C' and power_type == '0V': # set IO4 and IO2 to 0
            bit_mask &= 0xEB # 1110 1011
            
        ret, data = self.i2c_write_read(self.pca9557pw_addr, [0x01, bit_mask], 0, port=1)
        if ret is not True:
            logger.error("PCA9557PW write failed.")
            raise ValueError("PCA9557PW write failed.")
        
        return True
    
    def expand_io_init(self):
        
        '''
        @brief: Initialize expand IO.
        @return: Return True if success, False otherwise.
        '''
        
        ret, data = self.i2c_find_slave(port=1)
        if ret is not True:
            logger.error("I2C find slave failed.")
            raise ValueError("I2C find slave failed.")
        
        if self.pca9557pw_addr not in [int(slave_id, 16) for slave_id in data]:
            logger.error("PCA9557PW not found.")
            raise ValueError("PCA9557PW not found.")
        
        if not self._expand_io_init_status:
        
            reg_list = [
                [0x02, 0x00], # set all pins as normal polarity
                [0x03, 0xFF], # set all pins as input mode
                [0x01, 0x00],
            ]
            
            for reg in reg_list:
                ret, data = self.i2c_write_read(self.pca9557pw_addr, reg, 0, port=1)
                if ret != True:
                    logger.error("PCA9557PW init failed.")
                    raise ValueError("PCA9557PW init failed.")
                
            self._expand_io_init_status = True
            self._expand_io_mode_bitmask = 0xFF
            
        return True
            
    def expand_io_config(self, pin: int, mode: int):
        
        '''
        @brief: Configure expand IO pin mode.
        @param pin: Pin number. Only 0, 5, 6, 7 are supported.
        @param mode: Pin mode. 0: Input, 1: Output.
        '''
        
        if pin not in [0, 5, 6, 7]:
            raise ValueError("Invalid pin number. Only 0, 5, 6, 7 are supported.")
        
        if mode not in [0, 1]:
            raise ValueError("Invalid mode. Only 0 and 1 are supported.")
        
        if not self._expand_io_init_status:
            self.expand_io_init()
            
        ret, data = self.i2c_write_read(self.pca9557pw_addr, [0x03], 1, port=1)
        if ret is not True:
            logger.error("PCA9557PW read failed.")
            raise ValueError("PCA9557PW read failed.")
    
        bit_mask = data[0]
        bit_mask = (bit_mask & ~(1 << pin)) if mode == 1 else (bit_mask | (1 << pin))
        
        ret, data = self.i2c_write_read(self.pca9557pw_addr, [0x03, bit_mask], 0, port=1)
        if ret is not True:
            logger.error("PCA9557PW write failed.")
            raise ValueError("PCA9557PW write failed.")
        
        self._expand_io_mode_bitmask = bit_mask
        
        return True
        
    
    def expand_io_write_read(self, pin: int, level: int = None):
        
        '''
        @brief: Write or read level of expand IO pin.
        @param pin: Pin number. Only 0, 5, 6, 7 are supported.
        @param level: Level to write. 0: Low, 1: High. None in default for read mode.
        '''
        
        if pin not in [0, 5, 6, 7]:
            raise ValueError("Invalid pin number. Only 0, 5, 6, 7 are supported.")
        
        if level not in [0, 1] and level != None:
            raise ValueError("Invalid level. Only 0 and 1 are supported.")
        
        # read level
        if level == None:
            
            ret, data = self.i2c_write_read(self.pca9557pw_addr, 
                                            [0x00] if (self._expand_io_mode_bitmask >> pin) & 0x01 == 1 else [0x01], 
                                            1, port=1)
            if ret is not True:
                logger.error("PCA9557PW read failed.")
                raise ValueError("PCA9557PW read failed.")
            
            return True, level_
        
        # write level
        else:
            
            if self._expand_io_mode_bitmask >> pin & 0x01 == 1:
                raise ValueError("Pin is in input mode. Use expand_io_config() to set pin to output mode first.")
            
            ret, data = self.i2c_write_read(self.pca9557pw_addr, [0x00], 1, port=1)
            if ret is not True:
                logger.error("PCA9557PW read failed.")
                raise ValueError("PCA9557PW read failed.")
            
            if level == 1:
                bit_mask = data[0] | (1 << pin)
            else:
                bit_mask = data[0] & ~(1 << pin)
                
            ret, data = self.i2c_write_read(self.pca9557pw_addr, [0x01, bit_mask], 0, port=1)
            if ret is not True:
                logger.error("PCA9557PW write failed.")
                raise ValueError("PCA9557PW write failed.")
            
            return True, None