import sys
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                              QLabel, QGraphicsView, QGraphicsScene, QGraphicsItem)
from PySide6.QtCore import Qt, QTimer, QPointF, QRectF
from PySide6.QtGui import QPainter, QPen, QColor, QPolygonF, QBrush

import math
import numpy as np

class SensorVisualizer(QWidget):
    def __init__(self, dev, slave_addr):
        super().__init__()
        self.dev = dev
        self.slave_addr = slave_addr
        self.calibration = [0, 0, 0]  # 校准偏移量
        self.initUI()
        self.initSensor()
        self.startUpdates()

    def initUI(self):
        main_layout = QHBoxLayout()
        
        # 3D可视化区域
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setSceneRect(-300, -300, 600, 600)
        self.view.setAlignment(Qt.AlignCenter)
        
        # 创建3D立方体并居中
        self.cube = CubeItem3D()
        self.scene.addItem(self.cube)
        self.draw_reference_axes()

        # 数据面板
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

        self.setWindowTitle("ADXL345 3D Visualizer")
        self.setGeometry(300, 300, 800, 600)

    def initSensor(self):
        ret, data = self.dev.i2c_write_read(port=0, slave_id=self.slave_addr, 
                                          write_list=[0x00], read_length=1)
        assert data[0] == 0xE5, f"Expected 0xE5, got {data[0]:#04x}"

    def startUpdates(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateData)
        self.timer.start(50)

    def updateData(self):
        try:
            raw_data = get_gravity_data(slave_addr=self.slave_addr)
            if raw_data:
                x, y, z = self.apply_calibration(raw_data)
                self.x_label.setText(f"X: {x:.2f} mg")
                self.y_label.setText(f"Y: {y:.2f} mg")
                self.z_label.setText(f"Z: {z:.2f} mg")
                self.cube.updateOrientation(x, y, z)
        except Exception as e:
            print(f"数据更新错误: {str(e)}")

    def apply_calibration(self, raw_data):
        return (
            raw_data[0] - self.calibration[0],
            raw_data[1] - self.calibration[1],
            raw_data[2] - self.calibration[2]
        )

    def calibrate_sensor(self):
        try:
            samples = [get_gravity_data(slave_addr=self.slave_addr) for _ in range(20)]
            avg_x = sum(s[0] for s in samples) / len(samples)
            avg_y = sum(s[1] for s in samples) / len(samples)
            avg_z = sum(s[2] for s in samples) / len(samples)
            self.calibration = [avg_x, avg_y, avg_z - 1000]  # 假设理想z值为1000mg
        except Exception as e:
            print(f"校准失败: {str(e)}")

    def draw_reference_axes(self):
        """绘制带箭头的三维坐标系"""
        arrow_size = 15
        # X轴（红色）
        x_line = self.scene.addLine(0, 0, 200, 0, QPen(Qt.red, 2))
        x_arrow = QPolygonF([QPointF(200, 0), QPointF(200-arrow_size, -arrow_size/2),
                           QPointF(200-arrow_size, arrow_size/2)])
        self.scene.addPolygon(x_arrow, QPen(Qt.red), QBrush(Qt.red))
        
        # Y轴（绿色）
        y_line = self.scene.addLine(0, 0, 0, 200, QPen(Qt.green, 2))
        y_arrow = QPolygonF([QPointF(0, 200), QPointF(-arrow_size/2, 200-arrow_size),
                           QPointF(arrow_size/2, 200-arrow_size)])
        self.scene.addPolygon(y_arrow, QPen(Qt.green), QBrush(Qt.green))
        
        # Z轴（蓝色）
        z_line = self.scene.addLine(0, 0, -140, -140, QPen(Qt.blue, 2))
        z_arrow = QPolygonF([QPointF(-140, -140), QPointF(-140+arrow_size, -140+arrow_size/2),
                           QPointF(-140+arrow_size/2, -140+arrow_size)])
        self.scene.addPolygon(z_arrow, QPen(Qt.blue), QBrush(Qt.blue))

class CubeItem3D(QGraphicsItem):
    def __init__(self):
        super().__init__()
        self.size = 80
        self.rotation = {'x': 0, 'y': 0, 'z': 0}
        self.vertices = np.array([
            [-1, -1, -1], [1, -1, -1],
            [1, 1, -1], [-1, 1, -1],
            [-1, -1, 1], [1, -1, 1],
            [1, 1, 1], [-1, 1, 1]
        ], dtype=np.float64) * self.size
        
        self.edges = [
            (0,1), (1,2), (2,3), (3,0),
            (4,5), (5,6), (6,7), (7,4),
            (0,4), (1,5), (2,6), (3,7)
        ]
        self.faces = [
            (0,1,2,3), (4,5,6,7), (0,1,5,4),
            (2,3,7,6), (0,3,7,4), (1,2,6,5)
        ]
        self.face_colors = [
            QColor(255,0,0,100), QColor(0,255,0,100),
            QColor(0,0,255,100), QColor(255,255,0,100),
            QColor(255,0,255,100), QColor(0,255,255,100)
        ]

    def boundingRect(self):
        return QRectF(-200, -200, 400, 400)

    def updateOrientation(self, x, y, z):
        """将加速度转换为旋转角度（改进版本）"""
        self.rotation['x'] = np.degrees(np.arctan2(y, z))
        self.rotation['y'] = np.degrees(np.arctan2(x, z))
        self.update()

    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(Qt.black, 2))
        
        # 计算旋转矩阵（使用四元数避免万向节锁）
        rx = np.radians(self.rotation['x'])
        ry = np.radians(self.rotation['y'])
        
        # X轴旋转矩阵
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        
        # Y轴旋转矩阵
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        
        # 合成旋转矩阵并应用透视投影
        rotated_vertices = np.dot(self.vertices, np.dot(Rx, Ry))
        projected = []
        for v in rotated_vertices:
            # 透视投影参数
            f = 500  # 焦距
            z_offset = 800  # z轴偏移量
            x = v[0] * f / (v[2] + z_offset + f)
            y = v[1] * f / (v[2] + z_offset + f)
            projected.append((x, y))

        # 绘制面
        for i, face in enumerate(self.faces):
            polygon = QPolygonF()
            for vi in face:
                polygon.append(QPointF(projected[vi][0], projected[vi][1]))
            painter.setBrush(self.face_colors[i])
            painter.drawPolygon(polygon)

        # 绘制边
        for edge in self.edges:
            p1 = projected[edge[0]]
            p2 = projected[edge[1]]
            painter.drawLine(QPointF(*p1), QPointF(*p2))

# 修改主程序部分
if __name__ == "__main__":
    # 初始化传感器
    import sys, os
    sys.path.append(r'F:\gitlab\mxdbg\src')
    from mxESP32Debugger.debugger import Dbg as MXDBG
    import toml
    from loguru import logger
    import struct
    
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
    
    def get_gravity_data(slave_addr=0x53):
        ret, data = dev.i2c_write_read(port=0, slave_id=slave_addr, write_list=[0x32], read_length=6)
        if ret != True:
            print("Error reading data")
            return None

        data = bytearray(data)

        x = struct.unpack("<h", data[0:2])[0] * 3.9
        y = struct.unpack("<h", data[2:4])[0] * 3.9
        z = struct.unpack("<h", data[4:6])[0] * 3.9

        return x, y, z

    # 创建GUI应用
    app = QApplication(sys.argv)
    window = SensorVisualizer(dev, slave_addr)
    window.show()
    sys.exit(app.exec())