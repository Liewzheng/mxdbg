import unittest
from mxdbg import MXDBG

class TestMXDBG(unittest.TestCase):

    def setUp(self):
        """初始化 MXDBG 对象并连接设备。"""
        self.mxdbg = MXDBG()

    def test_i2c_write_read(self):
        """测试 I2C 写和读功能失败情况。"""
        with self.assertRaises(ValueError):
            self.mxdbg.i2c_write_read(slave_id=0x01, write_list=[0x02], read_length=1)
        with self.assertRaises(ValueError):
            self.mxdbg.i2c_write_read(slave_id=0x18, write_list=[0x02], read_length=1, port=2)
        with self.assertRaises(ValueError):
            self.mxdbg.i2c_write_read(slave_id=0x01, write_list=[], read_length=0, port=0)

    def test_gpio_config_valid(self):
        """测试 GPIO 配置的有效情况。"""
        ret = self.mxdbg.gpio_config(pin=33, mode=2, pull_up=1, pull_down=0)
        self.assertTrue(ret, "GPIO 配置失败")
        
    def test_gpio_config_invalid_mode(self):
        """测试 GPIO 配置的无效模式情况。"""
        with self.assertRaises(ValueError):
            self.mxdbg.gpio_config(pin=33, mode=99, pull_up=1, pull_down=0)
    
    def test_gpio_config_invalid_pull(self):
        """测试 GPIO 配置的无效上下拉情况。"""
        with self.assertRaises(ValueError):
            self.mxdbg.gpio_config(pin=33, mode=2, pull_up=99, pull_down=0)
        with self.assertRaises(ValueError):
            self.mxdbg.gpio_config(pin=33, mode=2, pull_up=1, pull_down=99)
            
    def test_spi_write_read(self):
        """测试 SPI 写和读功能。"""
        success, data = self.mxdbg.spi_write_read([0x00], 1)
        self.assertTrue(success, "SPI 写读操作失败")
        self.assertIsInstance(data, list, "返回数据类型错误")
        self.assertEqual(len(data), 1, "返回数据长度不正确")
        self.assertEqual(data[0], 0x00, "返回数据内容不正确")

    def test_pwm_config(self):
        """测试 PWM 配置。"""
        result, _ = self.mxdbg.pwm_config(pin=16, freq=10000, duty=0.5, channel=0)
        self.assertTrue(result, "PWM 配置失败")

    def test_usb_config(self):
        """测试 USB 配置。"""
        success, _ = self.mxdbg.usb_config(crc_enable=False)
        self.assertTrue(success, "USB 配置失败")

    def tearDown(self):
        """测试结束后断开设备连接。"""
        self.mxdbg.disconnect()

if __name__ == "__main__":
    unittest.main()
