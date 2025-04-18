import sys
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                              QLabel, QGraphicsView, QGraphicsScene, QGraphicsItem)
from PySide6.QtCore import Qt, QTimer, QPointF, QRectF
from PySide6.QtGui import QPainter, QPen, QColor, QTransform

# 保留原有传感器代码
class SensorVisualizer(QWidget):
    def __init__(self, dev, slave_addr):
        super().__init__()
        self.dev = dev
        self.slave_addr = slave_addr
        self.initUI()
        self.initSensor()
        self.startUpdates()

    def initUI(self):
        # 主布局
        main_layout = QHBoxLayout()
        
        # 3D可视化区域
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.cube = CubeItem()
        self.scene.addItem(self.cube)
        
        # 数据显示区域
        data_layout = QVBoxLayout()
        self.x_label = QLabel("X: 0.00 mg")
        self.y_label = QLabel("Y: 0.00 mg")
        self.z_label = QLabel("Z: 0.00 mg")
        data_layout.addWidget(self.x_label)
        data_layout.addWidget(self.y_label)
        data_layout.addWidget(self.z_label)
        
        main_layout.addWidget(self.view, 3)
        main_layout.addLayout(data_layout, 1)
        self.setLayout(main_layout)
        
        # 窗口设置
        self.setWindowTitle("ADXL345 3D Visualizer")
        self.setGeometry(300, 300, 800, 600)

    def initSensor(self):
        # 传感器初始化代码（与原始代码相同）
        ret, data = self.dev.i2c_write_read(port=0, slave_id=self.slave_addr, 
                                          write_list=[0x00], read_length=1)
        assert data[0] == 0xE5, f"Expected 0xE5, got {data[0]:#04x}"

    def startUpdates(self):
        # 定时器更新数据
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateData)
        self.timer.start(50)  # 20 FPS

    def updateData(self):
        # 获取并显示数据
        data = get_gravity_data(slave_addr=self.slave_addr)
        if data:
            x, y, z = data
            self.x_label.setText(f"X: {x:.2f} mg")
            self.y_label.setText(f"Y: {y:.2f} mg")
            self.z_label.setText(f"Z: {z:.2f} mg")
            self.cube.updateOrientation(x, y, z)

class CubeItem(QGraphicsItem):
    def __init__(self):
        super().__init__()
        self.rotation = {'x': 0, 'y': 0, 'z': 0}
        self.size = 100
        
    def boundingRect(self):
        return QRectF(-self.size, -self.size, 2*self.size, 2*self.size)

    def updateOrientation(self, x, y, z):
        # 加速度转换为旋转角度（简化处理）
        self.rotation['x'] = y / 1000 * 45  # 示例转换比例
        self.rotation['y'] = x / 1000 * 45
        self.rotation['z'] = z / 1000 * 45
        self.update()

    def paint(self, painter, option, widget):
        # 绘制3D立方体
        painter.setPen(QPen(Qt.black, 2))
        transform = QTransform()
        
        # 应用3D旋转（简化2D投影）
        transform.translate(self.size/2, self.size/2)
        transform.rotate(self.rotation['x'], Qt.XAxis)
        transform.rotate(self.rotation['y'], Qt.YAxis)
        transform.rotate(self.rotation['z'], Qt.ZAxis)
        painter.setTransform(transform)

        # 绘制立方体线框
        half = self.size / 2
        points = [
            QPointF(-half, -half),
            QPointF(half, -half),
            QPointF(half, half),
            QPointF(-half, half),
            QPointF(-half, -half)
        ]
        painter.drawPolygon(points)
        
        # 添加深度线
        depth = half * 0.8
        painter.drawLine(QPointF(-half, -half), QPointF(-half + depth, -half - depth))
        painter.drawLine(QPointF(half, -half), QPointF(half + depth, -half - depth))
        painter.drawLine(QPointF(half, half), QPointF(half + depth, half - depth))

# 修改主程序部分
if __name__ == "__main__":
    # 初始化传感器
    import sys, os, ctypes
    sys.path.append(r'F:\gitlab\mxdbg\src')
    from mxESP32Debugger.debugger import Dbg as MXDBG
    import toml
    from loguru import logger
    
    dev = MXDBG()
    dev.power_init()
    dev.power_control(communication_type="I2C", power_type="3V3")
    ret, data = dev.i2c_config(sda_pin=10, scl_pin=11, freq=400000, sda_pullup=True, scl_pullup=True)
    
    with open(os.path.join(os.path.dirname(__file__), "adxl345.toml"), "r", encoding='utf-8') as f:
        config = toml.load(f)
        
    reg_list = config['register_list']
    slave_addr = config['address']
    
    ret, data = dev.i2c_write_read(port=0, slave_id=slave_addr, write_list=[0x00], read_length=1)
    assert data[0] == 0xE5, f"Expected 0xE5, got {data[0]:#04x}"
    
    for reg in reg_list:
        ret, data = dev.i2c_write_read(port=0, slave_id=slave_addr, write_list=reg, read_length=0)
        assert ret == True, f"Failed to write register {reg:#04x}"
    
    def get_gravity_data(slave_addr):
        ret, data = dev.i2c_write_read(port=0, slave_id=slave_addr, write_list=[0x32], read_length=6)
        if ret != True:
            print("Error reading data")
            return None

        data = bytearray(data)

        x = ctypes.c_int16.from_buffer_copy(data[0:2]).value * 3.9
        y = ctypes.c_int16.from_buffer_copy(data[2:4]).value * 3.9
        z = ctypes.c_int16.from_buffer_copy(data[4:6]).value * 3.9

        return x, y, z

    # 创建GUI应用
    app = QApplication(sys.argv)
    window = SensorVisualizer(dev, slave_addr)
    window.show()
    sys.exit(app.exec())