'''
* Copyright (c) 2024 MixoSense Technology Ltd <contact@mixosense.com>.
*
* All rights are reserved.
* Proprietary and confidential.
* Unauthorized copying of this file, via any medium is strictly prohibited.
* Any use is subject to an appropriate license granted by MixoSense Technology
* Ltd.
'''

import ctypes
import os
import re
import sys
import time
from types import MappingProxyType

import serial.tools.list_ports
import toml
from loguru import logger
from serial import Serial

from mxdbg.__version__ import __version__


class MXDBG:

    def __init__(self, *args, **kwargs):
        self.__client = None
        self.__pwm_states = [False, False, False]
        self.__crc_enable = True
        self.__mxdbg_header_file = None
        self.__mxdbg_toml_path = None
        
        self.version, self.task_cmd, self.__error_map = self.__parse_and_map()

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

        self.__all_valid_pins = [-1, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
                                14, 15, 16, 17, 18, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42]
        self.__pwm_used_pins = [16, 17, 18]
        self.__i2c_used_pins = [10, 11]
        self.__spi_used_pins = [12, 13, 14, 15]
        self.__gpio_used_pins = set()

        self.__gpio_valid_pins = None
        self.__i2c_valid_pins = None
        self.__spi_valid_pins = None
        self.__pwm_valid_pins = None
        
        self.__pca9557pw_addr = 0x18
        self.__tca9555pwr_addr = 0x20
        self.__extboard_version = None
        
        self.__expand_io_init_status = False
        self.__power_init_status = False
        self.__expand_io_mode_bitmask = 0x0000

        self.connect(**kwargs)

    @property
    def pwm_valid_pins(self) -> list:
        if self.__pwm_valid_pins is None:
            self.__pwm_valid_pins = list(set(self.__all_valid_pins) - set(self.__gpio_used_pins) -
                                        set(self.__spi_used_pins) - set(self.__i2c_used_pins))
        return self.__pwm_valid_pins

    @property
    def i2c_valid_pins(self) -> list:
        if self.__i2c_valid_pins is None:
            self.__i2c_valid_pins = list(set(self.__all_valid_pins) - set(self.__pwm_used_pins) -
                                        set(self.__spi_used_pins) - set(self.__gpio_used_pins))
        return self.__i2c_valid_pins

    @property
    def spi_valid_pins(self) -> list:
        if self.__spi_valid_pins is None:
            self.__spi_valid_pins = list(set(self.__all_valid_pins) - set(self.__pwm_used_pins) -
                                        set(self.__i2c_used_pins) - set(self.__gpio_used_pins))
        return self.__spi_valid_pins

    @property
    def gpio_valid_pins(self) -> list:
        if self.__gpio_valid_pins is None:
            self.__gpio_valid_pins = list(set(self.__all_valid_pins) - set(self.__pwm_used_pins) -
                                         set(self.__spi_used_pins) - set(self.__i2c_used_pins))
        return self.__gpio_valid_pins

    def __mark_pwm_used(self, pin:int) -> None:

        if pin == -1:
            logger.warning("PWM is not supported to set pin to -1.")
            return

        if pin in self.pwm_valid_pins:
            self.__pwm_used_pins.append(pin)
            self.__pwm_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not a valid PWM pin or already used by other module.')

    def __mark_i2c_used(self, pin:int) -> None:

        if pin == -1:
            logger.warning("I2C is not supported to set pin to -1.")
            return

        if pin in self.i2c_valid_pins:
            self.__i2c_used_pins.append(pin)
            self.__i2c_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not a valid I2C pin or already used by other module.')

    def __mark_spi_used(self, pin:int) -> None:

        if pin == -1:
            return

        if pin in self.spi_valid_pins:
            self.__spi_used_pins.append(pin)
            self.__spi_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not a valid SPI pin or already used by other module.')

    def __mark_gpio_used(self, pin:int) -> None:

        if pin == -1:
            logger.warning("GPIO is not supported to set pin to -1.")
            return

        if pin in self.gpio_valid_pins:
            self.__gpio_used_pins.add(pin)
            self.__gpio_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not a valid GPIO pin or already used by other module.')

    def mark_pin_free(self, pin:int) -> None:
        '''
        @brief check all the used pins if the pin was in it, then remove it.
        @param pin: pin number. `-1` is not supported. 
        '''

        if pin == -1:
            logger.warning("Pin is not supported to be set as -1.")
            raise ValueError("Pin is not supported to be set as -1.")

        if pin in self.__gpio_used_pins:
            self.__gpio_used_pins.remove(pin)
            self.__gpio_valid_pins = None
        elif pin in self.__spi_used_pins:
            self.__spi_used_pins.remove(pin)
            self.__spi_valid_pins = None
        elif pin in self.__i2c_used_pins:
            self.__i2c_used_pins.remove(pin)
            self.__i2c_valid_pins = None
        elif pin in self.__pwm_used_pins:
            self.__pwm_used_pins.remove(pin)
            self.__pwm_valid_pins = None
        else:
            raise ValueError(f'Pin {pin} is not used by any module. Or it is not a valid pin.')

    def connect(self, **kwargs) -> None:

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
            if sys.platform != 'win32':
                _port = f"/dev/{_port}"
            
            # use serial port to connect
            self.__client = Serial(_port, 115200, timeout=5, write_timeout=1, **kwargs)
            logger.info("Using serial port to connect. Port: {}".format(_port))

        except Exception as e:
            raise ValueError(f"Failed to connect: {e}")
        
        logger.info("Using ESP32-S3R8.")
        logger.info("Embedded software version: v{}.{}".format(self.version["MAJOR"], self.version["MINOR"]))
        logger.info(f"Library version: {__version__}.")

    def disconnect(self) -> None:
        self.__client.close()

    def __read(self, timeout=2) -> bytearray:

        data = bytearray()
        start_time = time.time()

        while True:

            # 检查超时
            if time.time() - start_time > timeout:
                if len(data) == 0:
                    raise TimeoutError("Data read timeout. No data received.")
                else:
                    raise TimeoutError("Data read timeout. Data received: {}".format(data))

            if self.__client.in_waiting > 0:

                temp_data = self.__client.read(self.__client.in_waiting)
                data += temp_data

            if len(data) >= 5:
                if self.__check_crc(data) and (data[:5] == bytearray("mxdbg", "utf-8")):
                    break

            time.sleep(0.01)

        return data

    def __write(self, data) -> None:
        self.__client.write(data)

    def __task_execute(self, cmd, data: list) -> tuple:

        write_data = self.__data_pack(cmd, data)

        try:
            self.__write(write_data)
        except Exception as e:
            raise ValueError(f"Failed to write data: {e}")

        read_data = self.__read()

        ret, temp_data = self.__data_unpack(cmd, read_data)

        return ret, temp_data

    def __data_decompose(self, data: int, bytes_num: int = 4) -> tuple:
        return [(data >> (8 * (bytes_num - 1 - i))) & 0xFF for i in range(bytes_num)]

    def __hexdump(self, data: bytearray, base_address=0x3fc9900c) -> None:

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

    def __calculate_crc(self, data: bytearray) -> bytearray:
        '''
        @brief Calculate CRC16 for the given data.
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

    def __check_crc(self, data: bytearray) -> bool:
        '''
        @brief Check CRC16 for the given data.
        @param data: Data to check CRC16.
        @return: True if CRC16 is correct, False otherwise.
        '''

        if not self.__crc_enable:
            return True

        high, low = self.__calculate_crc(data[:-2])

        # logger.debug(f"CRC Check: {high}, {low}, {data[-2]}, {data[-1]}")

        return True if ((high == data[-2]) and (low == data[-1])) else False

    def __data_pack(self, cmd: int, data: list) -> bool:
        '''
        @brief Package data with header and CRC16.
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

        high, low = self.__calculate_crc(temp_data)
        temp_data += bytearray([high, low])

        return temp_data

    def __data_unpack(self, cmd, data: bytearray) -> tuple:
        '''
        @brief Unpackage data.
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
        assert self.__check_crc(temp_data), "CRC check failed."

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
    
    def __parse_version(self, lines) -> tuple:
        """Parse VERSION_MAJOR and VERSION_MINOR from header file."""
        try:
            major_pattern = re.compile(r"#define\s+VERSION_MAJOR\s+(\d+)")
            minor_pattern = re.compile(r"#define\s+VERSION_MINOR\s+(\d+)")

            for line in lines:
                major_match = major_pattern.search(line)
                if major_match:
                    version_major = int(major_match.group(1)) 

                minor_match = minor_pattern.search(line)
                if minor_match:
                    version_minor = int(minor_match.group(1))
        except Exception as e:
            raise ValueError(f"Error parsing version: {e}")
    
        return version_major, version_minor
    
    def __parse_task_cmd(self, lines) -> dict:
        """Parse TASK commands from header file."""
        task_cmd = {}
        try:
            cmd_pattern = re.compile(r"TASK_(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+),?")
            for line in lines:
                match = cmd_pattern.search(line)
                if match:
                    cmd_name = f"TASK_{match.group(1)}"
                    cmd_id = int(match.group(2), 0)
                    task_cmd[cmd_name] = cmd_id
        except Exception as e:
            raise ValueError(f"Error parsing task commands: {e}")
        return task_cmd
    
    def __parse_error_map(self, lines, task_cmd, version_major, version_minor) -> dict:
        """Parse error codes from header file, including MXDBG_ERR and ESPRESSIF_ERR."""
        error_map = {}
        try:
            # Regular expression to match ERR_SYNTHESIS with parameters
            err_synthesis_pattern = re.compile(
                r"#define\s+(MXDBG_ERR_\w+)\s+ERR_SYNTHESIS\((.+?)\)\s*/\*!\s*(.+?)\s*\*/"
            )

            # Regular expression to match simpler error patterns (e.g., ESPRESSIF_ERR)
            simple_error_pattern = re.compile(
                r"#define\s+(ESPRESSIF_ERR_\w+|ESPRESSIF_[A-Z_]+)\s+(0x[0-9A-Fa-f]+|\d+)\s*/\*!\s*(.+?)\s*\*/"
            )

            # Parse ERR_SYNTHESIS-based errors
            for line in lines:
                match = err_synthesis_pattern.search(line)
                if match:
                    error_name = match.group(1)  # e.g., "MXDBG_ERR_SPI_CONFIG_FAILED"
                    synthesis_args = match.group(2)  # e.g., "TASK_SPI_CONFIG, 0x00"
                    error_desc = match.group(3).strip()

                    # Parse ERR_SYNTHESIS arguments
                    args = synthesis_args.split(',')
                    if len(args) == 2:
                        task_name = args[0].strip()
                        ret_code = int(args[1].strip(), 0)

                        # Resolve task_id from task_cmd map
                        task_id = task_cmd.get(task_name, 0xFF)

                        # Compute error code based on ERR_SYNTHESIS logic
                        version_combined = (version_major << 24) | (version_minor << 16)
                        error_code = version_combined | (task_id << 8) | ret_code

                        error_map[error_name] = {'code': ctypes.c_int32(error_code).value, 'desc': error_desc.replace('< ', '')}
                    continue

                # Parse simpler error patterns (ESPRESSIF_ERR)
                match = simple_error_pattern.search(line)
                if match:
                    error_name = match.group(1)
                    error_code = int(match.group(2), 0)
                    error_desc = match.group(3).strip()
                    # error_map[error_name] = (ctypes.c_int32(error_code).value, error_desc.replace('< ', ''))
                    error_map[error_name] = {'code': ctypes.c_int32(error_code).value, 'desc': error_desc.replace('< ', '')}

        except Exception as e:
            raise ValueError(f"Error parsing error map: {e}")

        return error_map

    def __compute_error_code(self, version_major, version_minor, task_id, ret_code) -> int:
        """Compute the error code based on task_id and ret_code."""
        return ((version_major << 24) | (version_minor << 16) | (task_id << 8) | ret_code)

    def __save_to_toml(self, path, version, task_cmd, error_map) -> None:
        """Save parsed data to TOML file with additional annotations for version."""
        try:
            data = {
                'version': version,
                'task_cmd': task_cmd,
                'error_map': error_map
            }
            
            with open(path, 'w') as f:
                toml.dump(data, f)
                    
        except Exception as e:
            raise ValueError(f"Failed to save mxdbg.toml: {e}")
        
    def __parse_and_map(self) -> tuple:
        
        version = 0x00
        task_cmd = {}
        error_map = {}
        
        header_file = os.path.join(os.path.dirname(__file__), '../../espidf/main/mxdbg.h')
        if os.path.exists(header_file):
            self.__mxdbg_header_file = header_file
            self.__mxdbg_toml_path = os.path.join(os.path.dirname(__file__), 'mxdbg.toml')
        
        else:
            for f in os.listdir(os.path.dirname(__file__)):
                if f.endswith('.toml'):
                    if f == 'mxdbg.toml':
                        self.__mxdbg_toml_path = os.path.join(os.path.dirname(__file__), f)
                        break
                elif f.endswith('.h'):
                    if f == 'mxdbg.h':
                        self.__mxdbg_header_file = os.path.join(os.path.dirname(__file__), f)
                        break
                    
            if self.__mxdbg_toml_path is None and self.__mxdbg_header_file is None:
                raise FileNotFoundError("Please provide mxdbg.toml or mxdbg.h file to parse.")
            
        if self.__mxdbg_header_file:
            
            try:
                with open(self.__mxdbg_header_file, 'r') as f:
                    lines = f.readlines()
                
                version_major, version_minor = self.__parse_version(lines)
                version = {"MAJOR": version_major, "MINOR": version_minor}
                
                task_cmd = self.__parse_task_cmd(lines)
                        
                error_map = self.__parse_error_map(lines, task_cmd, version_major, version_minor)
            
                self.__save_to_toml(self.__mxdbg_toml_path, version, task_cmd, error_map)

            except Exception as e:
                raise ValueError(f"Failed to parse error map: {e}")
        
        else:
            try:
                with open(self.__mxdbg_toml_path, 'r') as f:
                    data = toml.load(f)

                version = data.get('version', version)
                task_cmd = data.get('task_cmd', task_cmd)
                error_map = data.get('error_map', error_map)
                
            except Exception as e:
                raise ValueError(f"Failed to load mxdbg.toml: {e}")
                    

        return version, task_cmd, error_map

    def __check_ret_code(self, cmd:int, ret:int) -> None:

        if ret != 0:

            error_code = self.__compute_error_code(self.version["MAJOR"], self.version["MINOR"], cmd, ret)
            error_desc = "Unknown error."
            for key in self.__error_map:
                if self.__error_map[key]["code"] == error_code:
                    error_desc = self.__error_map[key]["desc"]
                    break
            logger.error(f"Error code: 0x{ctypes.c_uint32(error_code).value:08X}, Description: {error_desc}")

    def i2c_find_slave(self, port: int = 0) -> tuple:
        
        '''
        @brief Find I2C slave devices.
        @param port: I2C port number. Default is `0`.
        @return: List of I2C slave devices.
        '''

        found_device_list = []
        for slave_id in range(0x04, 0x7F):
            try:
                ret, data = self.i2c_write_read(slave_id, [0x00], 0, port=port)
                if ret:
                    found_device_list.append(slave_id)
            except Exception:
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
                       slave_id_10_bit: bool = False) -> tuple:
        '''
        @brief Write and read data from I2C slave device. The default I2C pin is SDA: `10`, SCL: `11`.
        @param slave_id: I2C slave device address. (e.g., `0x04`.)
        @param write_list: Data to write to I2C slave device. (It should be a list of bytes. e.g., `[0x00, 0x01]` or `[]`.)
        @param read_length: Length of data to read from I2C slave device. 
        @param port: I2C port number.( It should be `0` (in default) or `1`.)
        @param slave_id_10_bit: True if 10-bit slave address, False otherwise.
        @return: Data read from I2C slave device.
        '''
        
        if port not in [0, 1]:
            logger.error("Invalid port number, the available port number is 0 or 1.")
            raise ValueError("Invalid port number, the available port number is 0 or 1.")
        
        if slave_id < 0x04 or slave_id > 0x07FF:
            logger.error("Invalid slave, the available slave id is between 0x04 and 0x07FF.")
            raise ValueError("Invalid slave, the available slave id is between 0x04 and 0x07FF.")

        # 构造要发送的数据包
        slave_id = slave_id if not slave_id_10_bit else slave_id & 0x07FF
        wirte_len = len(write_list)
        read_len = read_length
        
        if wirte_len <= 0 and read_len <= 0:
            logger.error("Write length and read length should be greater than 0.")
            raise ValueError("Write length and read length should be greater than 0.")

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
        ret, data = self.__task_execute(self.task_cmd["TASK_I2C_WRITE_READ"], i2c_data_temp)

        data = list(data) if data is not None else None

        if ret != 0:
            # logger.error(f"Error: {ret}")
            return False, None
        else:
            return True, data

    def i2c_config(self,
                   port: int = 0, freq: int = 400000,
                   sda_pin: int = 10, scl_pin: int = 11,
                   sda_pullup: bool = True, scl_pullup: bool = True) -> tuple:
        '''
        @brief Configure I2C port. ***Please do not modify the configuration of PORT1***.
        @param port: I2C port number.( It should be `0` (in default) or `1`.)
        @param freq: I2C frequency. (Default is `400000`.)
        @param sda_pin: SDA pin number. (Default is `10`.)
        @param scl_pin: SCL pin number. (Default is `11`.)
        @param sda_pullup: SDA pull up resistor. (`True`: enable, `False`: disable.)
        @param scl_pullup: SCL pull up resistor. (`True`: enable, `False`: disable.)
        @return: Return True if success, False otherwise.
        '''

        if port not in [0, 1]:
            logger.error("Invalid port number.")
            return False

        self.__i2c_valid_pins = None

        if (sda_pin not in self.i2c_valid_pins) or (scl_pin not in self.i2c_valid_pins):
            logger.error("Invalid pin number.")
            return False, None
        elif sda_pin in self.__i2c_used_pins and scl_pin in self.__i2c_used_pins:
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

        ret, data = self.__task_execute(self.task_cmd["TASK_I2C_CONFIG"], i2c_data_temp)

        if ret != 0:
            logger.error(f"Error: {ret}")
            return False, None
        else:
            for pin in self.__i2c_used_pins:
                self.mark_pin_free(pin)
            for pin in [sda_pin, scl_pin]:
                self.__mark_i2c_used(pin)
            return True, None

    def gpio_write_read(self, pin: int, level: int = None) -> tuple:
        '''
        @brief Write or read GPIO pin. This API will read GPIO's level if you set `level` to `None`, otherwise it will be in write mode.
        @param pin: GPIO pin number.
        @param level: GPIO level. (`0`: Low, `1`: High. `None` in default.)
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

        ret, data = self.__task_execute(self.task_cmd["TASK_GPIO_WRITE_READ"], gpio_data_temp)

        if ret == 0:
            return True, None if (operation == 0) else data[0]
        else:
            logger.error(f"Error: {ret}")
            return False, None

    def gpio_config(self, pin: int, mode: int, pull_up: bool, pull_down: bool) -> bool:
        '''
        @brief Configure GPIO pin.
        @param pin: GPIO pin number.
        @param mode: GPIO mode. (`0x00`: Disable, `0x01`: Input, `0x02`: Output, `0x03`: Input/Output, `0x06`: Output Open Drain, `0x07`: Input/Output Open Drain)
        @param pull_up: Pull up resistor. (`True`: Enable, `False`: Disable.)
        @param pull_down: Pull down resistor. (`True`: Enable, `False`: Disable.)
        @return: Return value. 0: Success, other: Error.
        '''

        self.__gpio_valid_pins = None

        if pin not in self.gpio_valid_pins:
            logger.error("Invalid pin number.")
            raise ValueError("Invalid pin number.")
        
        # assert mode in self.gpio_mode, "Invalid mode."
        if mode not in [v for k, v in self.gpio_mode.items()]:
            logger.error("Invalid mode.")
            raise ValueError("Invalid mode.")

        pin = ctypes.c_uint8(pin).value
        mode = ctypes.c_uint8(mode).value
        pull_up = ctypes.c_uint8(0x01 if pull_up else 0x00).value
        pull_down = ctypes.c_uint8(0x01 if pull_down else 0x00).value

        gpio_data_temp = [pin, mode, pull_up, pull_down]
        ret, data = self.__task_execute(self.task_cmd["TASK_GPIO_CONFIG"], gpio_data_temp)

        if ret != 0:
            logger.error(f"Error: {ret}")
            return False
        else:
            self.__mark_gpio_used(pin)
            return True

    def spi_write_read(self, write_list: list, read_length: int) -> tuple:
        '''
        @brief Write and read data from SPI slave device. (The default SPI pins are MISO: `12`, MOSI: `13`, SCLK: `14`, CS: `15`.)
        @param write_list: Data to write to SPI slave device. (It should be a list of bytes. e.g., `[0x00, 0x01]` or `[]`.)
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

        ret, data = self.__task_execute(self.task_cmd["TASK_SPI_WRITE_READ"], spi_data_temp)

        self.__check_ret_code(self.task_cmd["TASK_SPI_WRITE_READ"], ret)

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
                   ) -> tuple:
        '''
        @brief Configure SPI.
        @param miso_io_num: MISO pin number. (Default is `12`. Set to `-1` if not used.)
        @param mosi_io_num: MOSI pin number. (Default is `13`.)
        @param sclk_io_num: SCLK pin number.( Default is `14`.)
        @param cs_io_num: CS pin number. (Default is 15. Set to `-1` if not used.)
        @param quadwp_io_num: QUADWP pin number. (Default is `-1`.)
        @param quadhd_io_num: QUADHD pin number. (Default is `-1`.)
        @param data4_io_num: DATA4 pin number. (Default is `-1`.)
        @param data5_io_num: DATA5 pin number. (Default is `-1`.)
        @param data6_io_num: DATA6 pin number. (Default is `-1`.)
        @param data7_io_num: DATA7 pin number. (Default is `-1`.)
        @param max_transfer_sz: Maximum transfer size. (Default is `4096`.)
        @param common_bus_flags: Common bus flags. (Default is `0`.)
        @param isr_cpu_id: ISR CPU ID. (Default is `0`.)
        @param intr_flags: Interrupt flags. (Default is `0`.)
        @param command_bits: Command bits. (Default is `0`.)
        @param address_bits: Address bits. (Default is `0`.)
        @param dummy_bits: Dummy bits. (Default is `0`.)
        @param mode: SPI mode. (Default is `3`. Available values are `0`, `1`, `2`, `3`.)
        @param duty_cycle_pos: Duty cycle position. (Default is `0`.)
        @param cs_ena_pretrans: CS enable pretrans. (Default is `0`.)
        @param cs_ena_posttrans: CS enable posttrans. (Default is `0`.)
        @param freq: SPI frequency. (Default is `1000000`.)
        @param input_delay_ns: Input delay in ns. (Default is `0`.)
        @param device_interface_flags: Device interface flags. (Default is `0`. Set `(SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_3WIRE)` if 3-wired half duplex mode.
        @param queue_size: Queue size. Default is `7`.)
        @return: Return True if success, False otherwise.
        '''

        for pin in [mosi_io_num, miso_io_num, sclk_io_num, cs_io_num, quadhd_io_num, quadwp_io_num,
                    data4_io_num, data5_io_num, data6_io_num, data7_io_num]:
            if pin != -1 and ((pin not in self.spi_valid_pins) and (pin not in self.__spi_used_pins)):
                logger.error(f"Invalid pin number: {pin}")
                return False, None

        assert mode in [0, 1, 2, 3], "Invalid mode."
        assert common_bus_flags in self.spi_common_bus_flag.values(), "Invalid common bus flags."
        assert isr_cpu_id in [0, 1, 2], "Invalid ISR CPU ID."

        spi_config_data_temp = []
        spi_config_data_temp.extend(self.__data_decompose(mosi_io_num))
        spi_config_data_temp.extend(self.__data_decompose(miso_io_num))
        spi_config_data_temp.extend(self.__data_decompose(sclk_io_num))
        spi_config_data_temp.extend(self.__data_decompose(quadwp_io_num))
        spi_config_data_temp.extend(self.__data_decompose(quadhd_io_num))
        spi_config_data_temp.extend(self.__data_decompose(data4_io_num))
        spi_config_data_temp.extend(self.__data_decompose(data5_io_num))
        spi_config_data_temp.extend(self.__data_decompose(data6_io_num))
        spi_config_data_temp.extend(self.__data_decompose(data7_io_num))
        spi_config_data_temp.extend(self.__data_decompose(max_transfer_sz, 2))
        spi_config_data_temp.extend(self.__data_decompose(common_bus_flags))
        spi_config_data_temp.extend(self.__data_decompose(isr_cpu_id, 1))
        spi_config_data_temp.extend(self.__data_decompose(intr_flags))
        spi_config_data_temp.extend(self.__data_decompose(command_bits, 1))
        spi_config_data_temp.extend(self.__data_decompose(address_bits, 1))
        spi_config_data_temp.extend(self.__data_decompose(dummy_bits, 1))
        spi_config_data_temp.extend(self.__data_decompose(mode, 1))
        spi_config_data_temp.extend(self.__data_decompose(duty_cycle_pos, 2))
        spi_config_data_temp.extend(self.__data_decompose(cs_ena_pretrans, 2))
        spi_config_data_temp.extend(self.__data_decompose(cs_ena_posttrans, 1))
        spi_config_data_temp.extend(self.__data_decompose(freq))
        spi_config_data_temp.extend(self.__data_decompose(input_delay_ns))
        spi_config_data_temp.extend(self.__data_decompose(cs_io_num))
        spi_config_data_temp.extend(self.__data_decompose(device_interface_flags))
        spi_config_data_temp.extend(self.__data_decompose(queue_size))

        self.__hexdump(spi_config_data_temp)

        ret, data = self.__task_execute(self.task_cmd["TASK_SPI_CONFIG"], spi_config_data_temp)
        self.__check_ret_code(self.task_cmd["TASK_SPI_CONFIG"], ret)

        if ret == 0:
            for pin in self.__spi_used_pins:
                self.mark_pin_free(pin)
            for pin in [mosi_io_num, miso_io_num, sclk_io_num, cs_io_num, quadhd_io_num, quadwp_io_num,
                        data4_io_num, data5_io_num, data6_io_num, data7_io_num]:
                self.__mark_spi_used(pin)
            return True, None
        else:
            return False, None

    def spi_read_image(self) -> tuple:
        '''
        @brief Read image data from SPI slave device.
        @return: Image data.
        '''

        ret, data = self.__task_execute(self.task_cmd["TASK_SPI_READ_IMAGE"], [])

        if ret == 0:
            return True, data
        else:
            return False, None

    def pwm_run_stop(self, pwm_running_state: bool, channel: int = 0) -> tuple:
        '''
        @brief Run or stop PWM. It will generate a PWM signal with 10KHz frequency and 25% duty cycle in default.
        @param pwm_running_state: True to run PWM, False to stop PWM.
        @param channel: PWM channel. Default is 0. Available channels are 0, 1, 2.
        @return: Return True if success, False otherwise.
        '''

        if channel not in [0, 1, 2]:
            logger.error("Invalid channel number. Available channels are 0, 1, 2.")
            return False, None

        channel = ctypes.c_uint8(channel).value
        run_state = ctypes.c_uint8(pwm_running_state).value
        ret, data = self.__task_execute(self.task_cmd["TASK_PWM_RUN_STOP"], [channel, run_state])

        self.__check_ret_code(self.task_cmd["TASK_PWM_RUN_STOP"], ret)

        if pwm_running_state:
            self.__pwm_states[channel] = True
        else:
            self.__pwm_states[channel] = False

        return self.__pwm_states[channel], ret

    def pwm_config(self, pin: int = 16, freq: int = 10000, duty: float = 0.5, channel: int = 0, resolution_hz: int = 80000000) -> tuple:
        '''
        @brief Configure PWM.
        @param pin: PWM pin number. (Channel 0: `16` (in default); Channel 1: `17` (in default); Channel 2: `18` (in default).)
        @param freq: PWM frequency. Unit: Hz; (Default is `10000` (10KHz). Frequency should be far less than timer resolution.)
        @param duty: PWM duty cycle. (Default is `0.5` (50%). Duty cycle should be in the range of `0.0` (0%) to `1.0` (100%).)
        @param channel: PWM channel. (Default is `0`. Available channels are `0`, `1`, `2`.)
        @return: Return True if success, False otherwise.
        '''

        self.__pwm_valid_pins = None

        if pin not in self.pwm_valid_pins:
            logger.error("Invalid pin number.")
            return False, None

        if duty < 0 or duty > 1:
            logger.error("Invalid duty cycle. Duty cycle should be in the range of 0.0 to 1.0")
            return False, None

        if resolution_hz > 80_000_000:
            logger.error("Invalid timer resolution. Timer resolution should be less than 80MHz.")
            return False, None
        else:
            coeff = 80_000_000 // resolution_hz

        timer_resolution = resolution_hz

        if freq > timer_resolution:
            logger.error("Invalid frequency. Frequency should be less than timer resolution.")
            return False, None
        else:
            freq /= coeff
            freq = int(freq)

        timer_resolution = ctypes.c_uint32(timer_resolution).value
        period_ticks = ctypes.c_uint32(resolution_hz // freq).value
        duty_ticks = int(period_ticks * duty)
        
        assert 0 < period_ticks < 65535, "Invalid period ticks. Should be in the range of 1 to 65535."

        pin = ctypes.c_uint8(pin).value
        channel = ctypes.c_uint8(channel).value

        logger.debug(f"Period Ticks: {period_ticks}, Duty Ticks: {duty_ticks}")

        pwm_data_temp = [(channel & 0xFF),
                         (pin & 0xFF),

                         (period_ticks & 0xFF000000) >> 24, (period_ticks & 0x00FF0000) >> 16,
                         (period_ticks & 0x0000FF00) >> 8, period_ticks & 0x000000FF,

                         (duty_ticks & 0xFF000000) >> 24, (duty_ticks & 0x00FF0000) >> 16,
                         (duty_ticks & 0x0000FF00) >> 8, duty_ticks & 0x000000FF,
                         
                         (timer_resolution & 0xFF000000) >> 24, (timer_resolution & 0x00FF0000) >> 16,
                         (timer_resolution & 0x0000FF00) >> 8, timer_resolution & 0x000000FF
                         ]

        ret, _ = self.__task_execute(self.task_cmd["TASK_PWM_CONFIG"], pwm_data_temp)

        self.__check_ret_code(self.task_cmd["TASK_PWM_CONFIG"], ret)

        if ret == 0:
            self.__pwm_used_pins = [pin]

        if self.__pwm_states[channel]:
            self.pwm_run_stop(True, channel)

        for pin in self.__pwm_used_pins:
            self.mark_pin_free(pin)
        self.__mark_pwm_used(pin)

        return True, None

    def usb_config(self, crc_enable: bool = False) -> tuple:
        '''
        @brief Configure USB.
        @param crc_enable: (`True`: enable CRC, `False`: disable CRC.)
        @return: Return True if success, False otherwise.
        '''

        crc_enable = ctypes.c_uint8(crc_enable).value
        self.__crc_enable = crc_enable

        ret, data = self.__task_execute(self.task_cmd["TASK_USB_CONFIG"], [crc_enable])

        if ret == 0:
            return True, None
        else:
            return False, None
        
    def power_init(self) -> bool:
        
        '''
        @brief Initialize power.
        @return: Return True if success, False otherwise.
        '''

        if not self.__expand_io_init_status:
            ret = self.expand_io_init()
            if ret is not True:
                return False
            
        if not self.__power_init_status:
            
            match self.__extboard_version:
                
                case "v0.1":
                    
                    # set IO1 to IO3 as output
                    
                    ret, data = self.i2c_write_read(self.__pca9557pw_addr, [0x03], 1, port=1)
                    if ret is not True:
                        logger.error("PCA9557PW read failed.")
                        raise ValueError("PCA9557PW read failed.")
                    
                    bit_mask = data[0]
                    bit_mask &= 0xE1 # 1110 0001
                    
                    ret, data = self.i2c_write_read(self.__pca9557pw_addr, [0x03, bit_mask], 0, port=1)
                    if ret is not True:
                        logger.error("PCA9557PW write failed.")
                        raise ValueError("PCA9557PW write failed.")
                    
                    self.__expand_io_mode_bitmask = bit_mask
                    self.__power_init_status = True
                
                case "v0.2.1":
                
                    ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x06], 2, port=1)
                    if ret is not True:
                        logger.error("TCA9555PWR read failed.")
                        raise ValueError("TCA9555PWR read failed.")
                    
                    bit_mask = data[1] << 8 | data[0]
                    bit_mask &= 0xFC00 # 1111 1100 0000 0000
                    
                    ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x06, bit_mask & 0x00FF, (bit_mask & 0xFF00) >> 8], 0, port=1)
                    if ret is not True:
                        logger.error("TCA9555PWR write failed.")
                        raise ValueError("TCA9555PWR write failed.")
                    
                    self.__expand_io_mode_bitmask = bit_mask
                    self.__power_init_status = True
                
                case _:
                    
                    logger.error("Invalid extension board version.")
                    raise ValueError("Invalid extension board version.")
            
        else:
            logger.info("Power already initialized.")
        
        return True

    def power_control(self, communication_type:str="SPI", power_type:str='1V8') -> bool:
        
        '''
        @brief Control power for communication type.
        @param communication_type: Communication types which are supported to use level shifter. (`SPI` or `I2C` are available.)
        @param power_type: Power types which are supported to swich power. (`1V8`, `1V2` or `0V` are supported in ***Extboard v0.1***, while `3V3`, `1V8`, `1V2` or `0V` are supported in ***Extboard v0.2.1***.)
        @return: Return True if success, False otherwise.
        '''
        
        communication_type = communication_type.upper()
        power_type = power_type.upper()
        
        match self.__extboard_version:
            
            case "v0.1":
        
                if communication_type not in ["SPI", "I2C"]:
                    logger.error("Invalid communication type. Only SPI and I2C are supported.")
                    return False
                
                if power_type not in ["1V8", "1V2", "0V"]:
                    logger.error("Invalid power type. Only 1V8, 1V2 and 0V are supported.")
                    return False
                
                ret, data = self.i2c_write_read(self.__pca9557pw_addr, [0x01], 1, port=1)
                if ret is not True:
                    logger.error("PCA9557PW read failed.")
                    raise ValueError("PCA9557PW read failed.")
                
                bit_mask = data[0]
                
                match communication_type:
                    
                    case "SPI":
                        
                        match power_type:
                            
                            case "1V8": # IO3, bit 3
                                
                                bit_mask |= 0x08 # 0000 1000
                                bit_mask &= 0xFD # 1111 1101
                                
                            case "1V2": # IO1, bit 1
                                
                                bit_mask |= 0x02 # 0000 0010
                                bit_mask &= 0xF7 # 1111 0111
                                
                            case "0V": # set IO3 and IO1 to 0
                                
                                bit_mask &= 0xF5 # 1111 0101
                                
                            case "3V3":
                                
                                logger.error("Invalid power type. 3V3 is not supported in Extboard v0.1.")
                                return False
                            
                            case _:
                                    
                                logger.error("Invalid power type. Only 1V8, 1V2 and 0V are supported.")
                                return False
                    
                    case "I2C":
                        
                        match power_type:
                            
                            case "1V8": # IO4, bit 4
                                
                                bit_mask |= 0x10 # 0001 0000
                                bit_mask &= 0xFB # 1111 1011
                            
                            case "1V2": # IO2, bit 2
                                
                                bit_mask |= 0x04 # 0000 0100
                                bit_mask &= 0xEF # 1110 1111
                            
                            case "0V": # set IO4 and IO2 to 0
                                
                                bit_mask &= 0xEB # 1110 1011
                                
                            case "3V3":
                                
                                logger.error("Invalid power type. 3V3 is not supported in Extboard v0.1.")
                                return False
                            
                            case _:
                                
                                logger.error("Invalid power type. Only 1V8, 1V2 and 0V are supported.")
                                return False
                    
                ret, data = self.i2c_write_read(self.__pca9557pw_addr, [0x01, bit_mask], 0, port=1)
                if ret is not True:
                    logger.error("PCA9557PW write failed.")
                    raise ValueError("PCA9557PW write failed.")
                
            case "v0.2.1":
                
                if power_type not in ["3V3", "1V8", "1V2", "0V"]:
                    logger.error("Invalid power type. Only 3V3, 1V8, 1V2 and 0V are supported.")
                    return False
                
                ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x02], 2, port=1)
                if ret is not True:
                    logger.error("TCA9555PWR read failed.")
                    raise ValueError("TCA9555PWR read failed.")
                
                bit_mask = data[1] << 8 | data[0]
                
                '''
                REG 0x02
                | O 0.7 | O 0.6 | O 0.5   | O 0.4   | O 0.3   | O 0.2   | O 0.1   | O 0.0   |
                |-------|-------|---------|---------|---------|---------|---------|---------|
                | 1V8_2 | 1V8_1 | 3V3_I2C | 1V8_I2C | 1V2_I2C | 3V3_SPI | 1V8_SPI | 1V2_SPI |
                
                REG 0x03
                | O 1.7 | O 1.6 | O 1.5 | O 1.4 | O 1.3 | O 1.2 | O 1.1 | O 1.0 |
                |-------|-------|-------|-------|-------|-------|-------|-------|
                | IO17  | IO16  | IO15  | IO14  | IO13  | IO12  | 3V3_2 | 3V3_1 |
                '''
                
                match communication_type:
                    
                    case "SPI":
                        
                        match power_type:
                            
                            case "3V3":
                                
                                bit_mask |= 0x0004 # 0000 0000 0000 0100
                                bit_mask &= 0xFFFC # 1111 1111 1111 1100
                                
                            case "1V8":
                                
                                bit_mask |= 0x0002 # 0000 0000 0000 0010
                                bit_mask &= 0xFFFA # 1111 1111 1111 1010
                                
                            case "1V2":
                                
                                bit_mask |= 0x0001 # 0000 0000 0000 0001
                                bit_mask &= 0xFFF9 # 1111 1111 1111 1001
                                
                            case "0V":
                                
                                bit_mask &= 0xFFF8 # 1111 1111 1111 1000
                                
                            case _:
                                
                                logger.error("Invalid power type. Only 3V3, 1V8, 1V2 and 0V are supported.")
                                return False
                    
                    case "I2C":
                        
                        match power_type:
                            
                            case "3V3":
                                
                                bit_mask |= 0x0020 # 0000 0000 0010 0000
                                bit_mask &= 0xFFE7 # 1111 1111 1110 0111
                                
                            case "1V8":
                                
                                bit_mask |= 0x0010 # 0000 0000 0001 0000
                                bit_mask &= 0xFFD7 # 1111 1111 1101 0111
                                
                            case "1V2":
                                
                                bit_mask |= 0x0008 # 0000 0000 0000 1000
                                bit_mask &= 0xFFCF # 1111 1111 1100 1111
                                
                            case "0V":
                                
                                bit_mask &= 0xFFC7 # 1111 1111 1100 0111
                                
                            case _:
                                
                                logger.error("Invalid power type. Only 3V3, 1V8, 1V2 and 0V are supported.")
                                return False
                    
                    case _:
                        
                        logger.error("Invalid communication type. Only SPI and I2C are supported.")
                        return False
                    
                ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x02, bit_mask & 0x00FF, (bit_mask & 0xFF00) >> 8], 0, port=1)
                if ret is not True:
                    logger.error("TCA9555PWR write failed.")
                    raise ValueError("TCA9555PWR write failed.")
            
            case _:
                
                logger.error("Invalid extension board version.")
                raise ValueError("Invalid extension board version.")
        
        return True
    
    def expand_io_init(self) -> bool:
        
        '''
        @brief Initialize expand IO.
        @return: Return True if success, False otherwise.
        '''
        
        ret, board_version = self.get_extboard_version()
        if ret is not True:
            return False
        
        if not self.__expand_io_init_status:
            
            if board_version == "v0.1":
        
                reg_list = [
                    [0x02, 0x00], # set all pins as normal polarity
                    [0x03, 0xFF], # set all pins as input mode
                    [0x01, 0x00], # set all pins as low level
                ]
                
                for reg in reg_list:
                    ret, data = self.i2c_write_read(self.__pca9557pw_addr, reg, 0, port=1)
                    if not ret:
                        logger.error("PCA9557PW init failed.")
                        raise ValueError("PCA9557PW init failed.")
                    
                self.__expand_io_mode_bitmask = 0xFF
            
            elif board_version == "v0.2.1":
                
                reg_list = [
                    [0x06, 0xFF, 0xFF], # set all pins as input mode
                    [0x04, 0x00, 0x00], # disable polarity inversion
                    [0x02, 0x00, 0x00], # set all pins as low level
                ]
                
                for reg in reg_list:
                    reg, data = self.i2c_write_read(self.__tca9555pwr_addr, reg, 0, port=1)
                    if not ret:
                        logger.error("TCA9555PWR init failed.")
                        raise ValueError("TCA9555PWR init failed.")
                    
                self.__expand_io_mode_bitmask = 0xFFFF
        
        self.__expand_io_init_status = True
        return True
            
    def expand_io_config(self, pin: int, mode: int) -> bool:
        
        '''
        @brief Configure expand IO pin mode.
        @param pin: Expanded pin number on ExtBoard. (`IO0`, `IO5`, `IO6`, `IO7` are supported in ***Extboard v0.1***, while `IO6` (`1V8_1`), `IO7` (`1V8_2`), `IO10`(`3V3_1`), `IO11`(`3V3_2`), `IO12`, `IO13`, `IO14`, `IO15`, `IO16`, `IO17` are supported in ***Extboard v0.2.1***.)
        @param mode: Pin mode. `0`: Input, `1`: Output.
        '''
        
        if mode not in [0, 1]:
            raise ValueError("Invalid mode. Only 0 and 1 are supported.")
        
        if not self.__expand_io_init_status:
            self.expand_io_init()

        match self.__extboard_version:
            
            case "v0.1":
            
                if pin not in [0, 5, 6, 7]:
                    raise ValueError("Invalid pin number. Only 0, 5, 6, 7 are supported.")
                
                ret, data = self.i2c_write_read(self.__pca9557pw_addr, [0x03], 1, port=1)
                if ret is not True:
                    logger.error("PCA9557PW read failed.")
                    raise ValueError("PCA9557PW read failed.")
            
                bit_mask = data[0]
                bit_mask = (bit_mask & ~(1 << pin)) if mode == 1 else (bit_mask | (1 << pin))
                
                ret, data = self.i2c_write_read(self.__pca9557pw_addr, [0x03, bit_mask], 0, port=1)
                if ret is not True:
                    logger.error("PCA9557PW write failed.")
                    raise ValueError("PCA9557PW write failed.")
                
                self.__expand_io_mode_bitmask = bit_mask
                
            case "v0.2.1":
                
                '''
                REG 0x02
                | O 0.7      | O 0.6      | O 0.5   | O 0.4   | O 0.3   | O 0.2   | O 0.1   | O 0.0   |
                |------------|------------|---------|---------|---------|---------|---------|---------|
                | (IO7)1V8_2 | (IO6)1V8_1 | 3V3_I2C | 1V8_I2C | 1V2_I2C | 3V3_SPI | 1V8_SPI | 1V2_SPI |
                
                REG 0x03
                | O 1.7 | O 1.6 | O 1.5 | O 1.4 | O 1.3 | O 1.2 | O 1.1 | O 1.0 |
                |-------|-------|-------|-------|-------|-------|-------|-------|
                | IO17  | IO16  | IO15  | IO14  | IO13  | IO12  | 3V3_2 | 3V3_1 |
                '''
                
                if pin not in [12, 13, 14, 15, 16, 17]:
                    raise ValueError("Invalid pin number. Only 12, 13, 14, 15, 16, 17 are supported.")
                    
                ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x06], 2, port=1)
                if ret is not True:
                    logger.error("TCA9555PWR read failed.")
                    raise ValueError("TCA9555PWR read failed.")
                
                bit_mask = data[1] << 8 | data[0]
                if pin > 7:
                    pin -= 2
                    
                bit_mask = (bit_mask & ~(1 << pin)) if mode == 1 else (bit_mask | (1 << pin))
                
                ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x06, bit_mask & 0x00FF, (bit_mask & 0xFF00) >> 8], 0, port=1)
                if ret is not True:
                    logger.error("TCA9555PWR write failed.")
                    raise ValueError("TCA9555PWR write failed.")
                
                self.__expand_io_mode_bitmask = bit_mask
            
            case _:
                
                logger.error("Invalid extension board version.")
                raise ValueError("Invalid extension board version.")
        
        return True
        
    
    def expand_io_write_read(self, pin: any, level: int = None) -> tuple:
        
        '''
        @brief Write or read level of expand IO pin.
        @param pin: Expanded pin number on ExtBoard. (`IO0`, `IO5`, `IO6`, `IO7` are supported in ***Extboard v0.1***, while `IO6` (`1V8_1`), `IO7` (`1V8_2`), `IO10`(`3V3_1`), `IO11`(`3V3_2`), `IO12`, `IO13`, `IO14`, `IO15`, `IO16`, `IO17` are supported in ***Extboard v0.2.1***.)
        @param level: Level to write. `0`: Low, `1`: High. None in default for read mode.
        '''
        
        if level not in [0, 1] and level is not None:
            raise ValueError("Invalid level. Only 0 and 1 are supported.")

        match self.__extboard_version:
            
            case "v0.1":
                
                expand_pin_mapping = {
                    "IO0": 0,
                    "IO5": 5,
                    "IO6": 6,
                    "IO7": 7,
                }
                
                if isinstance(pin, str):
                    pin = pin.upper()
                    
                    if pin not in expand_pin_mapping.keys():
                        raise ValueError("Invalid pin number. Only IO0, IO5, IO6, IO7 are supported.")
                    else:
                        pin = expand_pin_mapping[pin]
        
                if pin not in [0, 5, 6, 7]:
                    raise ValueError("Invalid pin number. Only 0, 5, 6, 7 are supported.")
                
                # read level
                if level is None:
                    
                    ret, data = self.i2c_write_read(self.__pca9557pw_addr, 
                                                    [0x00] if (self.__expand_io_mode_bitmask >> pin) & 0x01 == 1 else [0x01], 
                                                    1, port=1)
                    if ret is not True:
                        logger.error("PCA9557PW read failed.")
                        raise ValueError("PCA9557PW read failed.")
                    
                    level_ = (data[0] >> pin) & 0x01
                    
                    return True, level_
                
                # write level
                else:
                    
                    if self.__expand_io_mode_bitmask >> pin & 0x01 == 1:
                        raise ValueError("Pin is in input mode. Use expand_io_config() to set pin to output mode first.")
                    
                    ret, data = self.i2c_write_read(self.__pca9557pw_addr, [0x00], 1, port=1)
                    if ret is not True:
                        logger.error("PCA9557PW read failed.")
                        raise ValueError("PCA9557PW read failed.")
                    
                    if level == 1:
                        bit_mask = data[0] | (1 << pin)
                    else:
                        bit_mask = data[0] & ~(1 << pin)
                        
                    ret, data = self.i2c_write_read(self.__pca9557pw_addr, [0x01, bit_mask], 0, port=1)
                    if ret is not True:
                        logger.error("PCA9557PW write failed.")
                        raise ValueError("PCA9557PW write failed.")
                    
                    return True, None
                
            case "v0.2.1":
                
                expand_pin_mapping = {
                    "IO6": 6,
                    "IO7": 7,
                    "IO10": 10,
                    "IO11": 11,
                    "IO12": 12,
                    "IO13": 13,
                    "IO14": 14,
                    "IO15": 15,
                    "IO16": 16,
                    "IO17": 17,
                    "1V8_1": 6,
                    "1V8_2": 7,
                    "3V3_1": 10,
                    "3V3_2": 11
                }
                
                if isinstance(pin, str):
                    pin = pin.upper()
                    
                    if pin not in expand_pin_mapping.keys():
                        raise ValueError("Invalid pin number. Only IO6, IO7, IO10, IO11, IO12, IO13, IO14, IO15, IO16, IO17, 1V8_1, 1V8_2, 3V3_1, 3V3_2 are supported.")
                    else:
                        pin = expand_pin_mapping[pin]
                
                if pin not in [6, 7, 10, 11, 12, 13, 14, 15, 16, 17]:
                    raise ValueError("Invalid pin number. Only 6, 7, 10, 11, 12, 13, 14, 15, 16, 17 are supported.")
                
                if pin > 7:
                    pin -= 2
                
                # read level
                if level is None:
                    
                    ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x00], 2, port=1)
                    if ret is not True:
                        logger.error("TCA9555PWR read failed.")
                        raise ValueError("TCA9555PWR read failed.")
                    
                    level_ = (data[1] << 8 | data[0]) >> pin & 0x01
                    
                    return True, level_
                
                # write level
                else:
                    if self.__expand_io_mode_bitmask >> pin & 0x01 == 1:
                        raise ValueError("Pin is in input mode. Use expand_io_config() to set pin to output mode first.")
                    
                    ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x00], 2, port=1)
                    if ret is not True:
                        logger.error("TCA9555PWR read failed.")
                        raise ValueError("TCA9555PWR read failed.")
                    
                    if level == 1:
                        bit_mask = (data[1] << 8 | data[0]) | (1 << pin)
                    else:
                        bit_mask = (data[1] << 8 | data[0]) & ~(1 << pin)
                        
                    ret, data = self.i2c_write_read(self.__tca9555pwr_addr, [0x02, bit_mask & 0x00FF, (bit_mask & 0xFF00) >> 8], 0, port=1)
                    if ret is not True:
                        logger.error("TCA9555PWR write failed.")
                        raise ValueError("TCA9555PWR write failed.")
                    
                    return True, None
            
            case _:
                
                logger.error("Invalid extension board version.")
                raise ValueError("Invalid extension board version.")
            
        
    def restart(self) -> bool:
        
        '''
        @brief Restart the device.
        '''
        
        ret, data = self.__task_execute(self.task_cmd["TASK_RESET_DEVICE"], [])
        if ret != 0:
            logger.error("Restart failed.")
            raise ValueError("Restart failed.")

        return True
    
    def get_extboard_version(self) -> tuple:
        
        '''
        @brief Get extension board version.
        @return: Extension board version.
        '''
        
        ret, data = self.i2c_find_slave(port=1)
        if ret is not True:
            
            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> BUG FIXED FOR v0.2.1 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            ret, data = self.i2c_config(port=1, freq=400000, sda_pin=41, scl_pin=42, sda_pullup=False, scl_pullup=False)
            logger.info("I2C config for extension board.")
            if ret is not True:
                logger.error("I2C config failed.")
                raise ValueError("I2C config failed.")
            
            ret, data = self.i2c_find_slave(port=1)
            if ret is not True:
                logger.info("No extension board found.")
                return False, None
            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> BUG FIXED FOR v0.2.1 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
        if self.__pca9557pw_addr in [int(i, 16) for i in data]:
            self.__extboard_version = "v0.1"
            logger.info("Extension board version: v0.1")

        elif self.__tca9555pwr_addr in [int(i, 16) for i in data]:
            self.__extboard_version = "v0.2.1"
            logger.info("Extension board version: v0.2.1")
            
        else:
            logger.error("Unknown extension board.")
            return False, None
        
        return True, self.__extboard_version
        