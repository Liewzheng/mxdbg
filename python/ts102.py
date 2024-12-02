# coding: utf-8

import serial
from serial.tools import list_ports
from loguru import logger
import struct

class TS102:
    
    def __init__(self):
        ports = list_ports.comports()
        
        self.com = None
        config = {
            "description": "USB-SERIAL CH340",
            "vid": "1A86",
            "pid": "7523",
            "baudrate": 9600,
            "timeout": 1,
            "bitwidth": serial.EIGHTBITS,
            "parity": serial.PARITY_NONE,
            "stopbits": serial.STOPBITS_ONE,
        }
        for p, desc, hwid in sorted(ports):
            # print("{}: {} [{}]".format(p, desc, hwid))
            if config["description"] in desc and config["vid"] in hwid and config["pid"] in hwid:
                self.com = p
                break
        
        if self.com is None:
            logger.error("No device found")
            raise Exception("No device found")
        else:
            logger.info("Found device at port: {}", self.com)
            
        try:
            self.port = serial.Serial(self.com, config["baudrate"], timeout=config["timeout"], bytesize=config["bitwidth"], parity=config["parity"], stopbits=config["stopbits"])
        except serial.SerialException as e:
            logger.error("Error opening serial port: {}", e)
            raise Exception("Error opening serial port")    
    
    def __del__(self):
        self.close()
    
    def close(self):
        self.port.close()
        
    @staticmethod
    def calculate_crc(data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc
    
    def read_temperature_data(self):
        # Modbus RTU Request to read 4 registers from address 0x0000
        request = bytearray([0x01, 0x03, 0x00, 0x00, 0x00, 0x04])
        crc = self.calculate_crc(request)
        request.append(crc & 0xFF)
        request.append((crc >> 8) & 0xFF)
        
        # Send request
        self.port.write(request)

        # Read response (9 bytes expected: address, function, byte count, data x 4, CRC x 2)
        response = self.port.read(13)
        if len(response) != 13:
            raise Exception("Incomplete response")

        # Validate CRC
        received_crc = struct.unpack('<H', response[-2:])[0]
        calculated_crc = self.calculate_crc(response[:-2])
        if received_crc != calculated_crc:
            raise Exception("CRC error")

        # Parse temperature data
        temp1_high = struct.unpack('>H', response[3:5])[0]
        temp1_low = struct.unpack('>H', response[5:7])[0]
        temp2_high = struct.unpack('>H', response[7:9])[0]
        temp2_low = struct.unpack('>H', response[9:11])[0]

        # Convert to temperatures using IEEE 754 format
        temp1_bytes = struct.pack('>HH', temp1_high, temp1_low)
        temp2_bytes = struct.pack('>HH', temp2_high, temp2_low)
        temp1 = struct.unpack('>f', temp1_bytes)[0]
        temp2 = struct.unpack('>f', temp2_bytes)[0]

        return temp1, temp2

    