import sys

import keyboard
import numpy as np
from PySide6.QtCore import QElapsedTimer, Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QLabel, QMainWindow, QVBoxLayout,
                               QWidget)

from mxESP32Debugger.debugger import Dbg as MXDBG


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the main window layout and label to show the image
        self.setWindowTitle("Live Image Display")
        
        # Image display label
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        
        # Frame rate display label
        self.fps_label = QLabel(self)
        self.fps_label.setAlignment(Qt.AlignCenter)
        self.fps_label.setStyleSheet("font-size: 20px; color: green;")
        
        # Set up layout and central widget
        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.fps_label)  # Add FPS label to layout
        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Initialize MXDBG device
        self.dev = MXDBG()
        ret, data = self.dev.usb_config(crc_enable=False)
        print(ret, data)

        ret, data = self.dev.spi_config(
            freq=6000000,
            miso_io_num=-1,
            cs_ena_pretrans=1,
            cs_ena_posttrans=1,
            device_interface_flags=(self.dev.spi_device["SPI_DEVICE_HALFDUPLEX"] | self.dev.spi_device["SPI_DEVICE_3WIRE"])
        )
        print(ret, data)

        # Set up the timer for continuous updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(100)  # Update every 100ms

        # Set up the elapsed timer to calculate FPS
        self.elapsed_timer = QElapsedTimer()
        self.elapsed_timer.start()
        self.frames = 0

    def update_image(self):
        """Fetch new image data and update the display."""
        ret, data = self.dev.spi_read_image()

        if len(data) == 900:
            # Convert the data into a numpy array and reshape it to 30x30
            data = np.array(data)
            data = data.reshape(30, 30)

            # Convert the numpy array to QImage
            qimage = self.numpy_to_qimage(data)

            # Resize image to 300x300 using SmoothTransformation (no loss)
            scaled_qimage = qimage.scaled(300, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)

            # Update the QLabel with the new image
            pixmap = QPixmap.fromImage(scaled_qimage)
            self.image_label.setPixmap(pixmap)

            # Increment frame count and update FPS every 1 second
            self.frames += 1
            if self.elapsed_timer.elapsed() >= 1000:  # Every second
                fps = self.frames
                self.fps_label.setText(f"FPS: {fps}")
                self.frames = 0  # Reset frame count
                self.elapsed_timer.restart()  # Restart the timer

        if keyboard.is_pressed('q'):
            self.close()

    def numpy_to_qimage(self, np_array):
        """Convert a numpy array to a QImage."""
        height, width = np_array.shape
        bytes_per_line = width
        return QImage(np_array.data, width, height, bytes_per_line, QImage.Format_Grayscale8)


if __name__ == "__main__":
    # Set up the application and the main window
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    sys.exit(app.exec())
