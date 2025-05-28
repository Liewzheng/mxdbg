import sys, struct
import numpy as np
from PySide6.QtCore import Qt, QTimer, QPointF, QRectF, QObject
from PySide6.QtGui import (QPainter, QPen, QColor, QTransform, 
                          QPolygonF, QVector3D, QMatrix4x4, QPainterPath)
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                              QLabel, QGraphicsView, QGraphicsScene, 
                              QGraphicsItem, QPushButton)

class PhysicsEngine:
    def __init__(self):
        self.velocity = QVector3D(0, 0, 0)
        self.position = QVector3D(0, 0, 0)
        self.accel_offset = QVector3D(0, 0, 0)
        self.damping = 0.98  # 空气阻尼系数
        self.floor_level = -500  # 地面高度
        self.restitution = 0.7  # 碰撞恢复系数

    def update(self, raw_accel, delta_time):
        # 应用校准并转换为m/s²
        calibrated_accel = raw_accel - self.accel_offset
        accel = calibrated_accel * 9.8 / 1000  # 转换为标准重力加速度

        # 欧拉积分计算运动
        self.velocity = self.velocity * self.damping + accel * delta_time
        new_position = self.position + self.velocity * delta_time

        # 碰撞检测与响应
        if new_position.y() < self.floor_level:
            new_position.setY(self.floor_level)
            self.velocity.setY(-self.velocity.y() * self.restitution)
            # 摩擦损耗
            self.velocity.setX(self.velocity.x() * 0.8)
            self.velocity.setZ(self.velocity.z() * 0.8)

        self.position = new_position
        return self.position

class SensorVisualizer(QWidget):
    def __init__(self, dev, slave_addr):
        super().__init__()
        self.dev = dev
        self.slave_addr = slave_addr
        self.physics = PhysicsEngine()
        self.initUI()
        self.initSensor()
        self.startUpdates()
        
        # 轨迹可视化相关
        self.trail_points = []
        self.max_trail_length = 50
        
    def initSensor(self):
        import os
        sys.path.append(r'F:\gitlab\mxdbg\src')
        from mxESP32Debugger.debugger import Dbg as MXDBG
        import toml
        
        self.dev = MXDBG()
        self.dev.power_init()
        self.dev.power_control(communication_type="I2C", power_type="3V3")
        ret, data = self.dev.i2c_config(sda_pin=10, scl_pin=11, freq=400000, sda_pullup=True, scl_pullup=True)
        
        with open(os.path.join(os.path.dirname(__file__), "adxl345.toml"), "r", encoding='utf-8') as f:
            config = toml.load(f)
            
        reg_list = config['register_list']
        slave_addr = config['address']
        
        ret, data = self.dev.i2c_write_read(port=0, slave_id=slave_addr, write_list=[0x00], read_length=1)
        assert data[0] == 0xE5, f"Expected 0xE5, got {data[0]:#04x}"
        
        for reg in reg_list:
            ret, data = self.dev.i2c_write_read(port=0, slave_id=slave_addr, write_list=reg, read_length=0)
            assert ret == True, f"Failed to write register {reg:#04x}"
            
    def get_gravity_data(self, slave_addr=0x53):
        ret, data = self.dev.i2c_write_read(port=0, slave_id=slave_addr, write_list=[0x32], read_length=6)
        if ret != True:
            print("Error reading data")
            return None

        data = bytearray(data)

        x = struct.unpack("<h", data[0:2])[0] * 3.9
        y = struct.unpack("<h", data[2:4])[0] * 3.9
        z = struct.unpack("<h", data[4:6])[0] * 3.9

        return x, y, z
    
    def startUpdates(self):
        """初始化并启动数据更新定时器"""
        try:
            self.timer = QTimer(self)
            self.timer.setInterval(50)
            if not hasattr(self, 'updateData'):
                raise AttributeError("Missing updateData method")
            self.timer.timeout.connect(self.updateData)
            self.timer.start()
        except Exception as e:
            print(f"Timer initialization failed: {str(e)}")
            # 此处可添加错误恢复逻辑

    def initUI(self):
        main_layout = QHBoxLayout()
        
        # 3D场景设置
        self.scene = QGraphicsScene()
        self.view = Graphics3DView(self.scene)
        self.scene.setSceneRect(-800, -600, 1600, 1200)
        
        # 创建可移动物体
        self.cube = DynamicCubeItem()
        self.scene.addItem(self.cube)
        self.draw_environment()
        
        # 控制面板
        control_panel = QVBoxLayout()
        self.pos_label = QLabel("Position: (0, 0, 0)")
        self.vel_label = QLabel("Velocity: (0, 0, 0)")
        btn_calibrate = QPushButton("校准传感器")
        btn_reset = QPushButton("重置模拟")
        btn_calibrate.clicked.connect(self.calibrate_sensor)
        btn_reset.clicked.connect(self.reset_simulation)
        
        control_panel.addWidget(self.pos_label)
        control_panel.addWidget(self.vel_label)
        control_panel.addWidget(btn_calibrate)
        control_panel.addWidget(btn_reset)
        
        main_layout.addWidget(self.view, 4)
        main_layout.addLayout(control_panel, 1)
        self.setLayout(main_layout)
        self.setWindowTitle("3D物理模拟系统")
        self.resize(1200, 800)

    def draw_environment(self):
        # 绘制三维网格地面
        grid_size = 1000
        step = 50
        pen = QPen(QColor(100, 100, 100, 80), 1)
        for x in range(-grid_size, grid_size, step):
            self.scene.addLine(x, -grid_size, x, grid_size, pen)
        for y in range(-grid_size, grid_size, step):
            self.scene.addLine(-grid_size, y, grid_size, y, pen)

        # 绘制坐标轴
        axis_size = 300
        self.scene.addLine(0, 0, axis_size, 0, QPen(Qt.red, 2))    # X轴
        self.scene.addLine(0, 0, 0, axis_size, QPen(Qt.green, 2))  # Y轴
        self.scene.addLine(0, 0, -axis_size, -axis_size, QPen(Qt.blue, 2)) # Z轴

    def updateData(self):
        raw_data = self.get_gravity_data(self.slave_addr)
        if raw_data:
            delta_time = 0.05  # 50ms时间步长
            accel = QVector3D(*raw_data)
            position = self.physics.update(accel, delta_time)
            
            # 更新3D物体位置
            self.cube.setPosition(position)
            
            # 更新轨迹
            self.trail_points.append(position)
            if len(self.trail_points) > self.max_trail_length:
                self.trail_points.pop(0)
            self.cube.updateTrail(self.trail_points)
            
            # 更新UI显示
            self.pos_label.setText(f"Position: {position.x():.1f}, {position.y():.1f}, {position.z():.1f}")
            self.vel_label.setText(f"Velocity: {self.physics.velocity.x():.1f}, "
                                  f"{self.physics.velocity.y():.1f}, "
                                  f"{self.physics.velocity.z():.1f}")

    def calibrate_sensor(self):
        samples = [self.get_gravity_data(self.slave_addr) for _ in range(50)]
        avg = np.mean(samples, axis=0)
        self.physics.accel_offset = QVector3D(*avg)
        print(f"校准完成，偏移量：{self.physics.accel_offset}")

    def reset_simulation(self):
        self.physics.position = QVector3D(0, 0, 0)
        self.physics.velocity = QVector3D(0, 0, 0)
        self.trail_points.clear()

class Graphics3DView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)
        # 初始化3D变换矩阵
        self.view_matrix = QMatrix4x4()
        self.projection_matrix = QMatrix4x4()
        self.updateTransform()
        
    def updateTransform(self):
        """更新3D变换矩阵"""
        # 视图矩阵
        self.view_matrix.setToIdentity()
        self.view_matrix.lookAt(
            QVector3D(0, 800, 1200),  # 摄像机位置
            QVector3D(0, 0, 0),       # 观察点
            QVector3D(0, 1, 0)        # 上方向
        )
        
        # 投影矩阵
        self.projection_matrix.setToIdentity()
        self.projection_matrix.perspective(
            45.0,                     # 视角
            self.width()/self.height(), # 宽高比
            0.1,                      # 近平面
            10000.0                   # 远平面
        )
    
    def get3DTransform(self):
        """获取组合后的变换矩阵"""
        return self.projection_matrix * self.view_matrix

    def resizeEvent(self, event):
        """窗口大小改变时更新投影"""
        self.updateTransform()
        super().resizeEvent(event)

class DynamicCubeItem(QGraphicsItem):
    def __init__(self):
        super().__init__()
        self.size = 80
        self.position = QVector3D()
        self.trail = []
        
        # 初始化立方体顶点
        self.vertices = [
            QVector3D(-1, -1, -1), QVector3D(1, -1, -1),
            QVector3D(1, 1, -1), QVector3D(-1, 1, -1),
            QVector3D(-1, -1, 1), QVector3D(1, -1, 1),
            QVector3D(1, 1, 1), QVector3D(-1, 1, 1)
        ]
        self.vertices = [v * self.size for v in self.vertices]
        
        self.edges = [
            (0,1), (1,2), (2,3), (3,0),
            (4,5), (5,6), (6,7), (7,4),
            (0,4), (1,5), (2,6), (3,7)
        ]

    def setPosition(self, pos):
        self.position = pos
        self.update()

    def updateTrail(self, points):
        self.trail = points
        self.update()

    def boundingRect(self):
        return QRectF(-1000, -1000, 2000, 2000)

    def paint(self, painter, option, widget):
        # 获取关联的 QGraphicsView
        view = self.scene().views()[0] if self.scene() else None
        if not view:
            return

        painter.save()
        try:
            # 应用视图变换
            painter.setWorldTransform(view.viewportTransform())
            
            # 绘制轨迹
            if len(self.trail) > 1:
                path = QPainterPath()
                start = self.project3D(view, self.trail[0])
                path.moveTo(start)
                for p in self.trail[1:]:
                    path.lineTo(self.project3D(view, p))
                painter.strokePath(path, QPen(Qt.yellow, 2))
            
            # 绘制立方体
            rotated_vertices = [self.position + v * self.size for v in [
                QVector3D(-1, -1, -1), QVector3D(1, -1, -1),
                QVector3D(1, 1, -1), QVector3D(-1, 1, -1),
                QVector3D(-1, -1, 1), QVector3D(1, -1, 1),
                QVector3D(1, 1, 1), QVector3D(-1, 1, 1)
            ]]
            
            # 绘制边
            painter.setPen(QPen(Qt.blue, 2))
            for edge in [
                (0,1), (1,2), (2,3), (3,0),
                (4,5), (5,6), (6,7), (7,4),
                (0,4), (1,5), (2,6), (3,7)
            ]:
                p1 = self.project3D(view, rotated_vertices[edge[0]])
                p2 = self.project3D(view, rotated_vertices[edge[1]])
                painter.drawLine(p1, p2)
        finally:
            painter.restore()

    def project3D(self, view, point):
        """使用 QGraphicsView 的变换进行投影"""
        # 将3D点转换为场景坐标
        scene_pos = QPointF(point.x(), point.y())
        
        # 转换为视图坐标
        view_pos = view.mapFromScene(scene_pos)
        return view_pos

# 传感器数据获取函数（需根据实际硬件调整）
# def get_gravity_data(slave_addr):
#     # 模拟数据（实际应从硬件获取）
#     return QVector3D(
#         np.random.normal(0, 50),  # X轴噪声
#         np.random.normal(0, 50),  # Y轴噪声
#         np.random.normal(1000, 50) # Z轴重力
#     )

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SensorVisualizer(None, 0x53)  # 传入实际设备对象
    window.show()
    sys.exit(app.exec())