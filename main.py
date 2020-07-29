# coding: utf-8

import sys, os

from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QThread
from PyQt5.QtSerialPort import QSerialPort
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtCore import QIODevice
from PyQt5.QtCore import QWaitCondition
from PyQt5.QtCore import QMutex
from PyQt5.QtCore import QByteArray
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import pyqtSignal


from PyQt5.QtCore import QDate
from PyQt5.QtCore import QTime
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtGui import QTextCursor


import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.animation as animation
from PyQt5.QtGui import QTextDocument
from PyQt5.QtGui import QTextBlock

import random
import time


buf_list = list()
temp_buf_list = list()
tmp_str = ''
done = False


class MyMplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(211, xlim=(0, 60), ylim=(-30, 30))
        self.axes2 = fig.add_subplot(212, xlim=(0, 60), ylim=(-30, 30))
        self.compute_initial_figure()
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

    def compute_initial_figure(self):
        pass


class AnimationWidget(QWidget):
    global temp_buf_list

    def __init__(self):
        QWidget.__init__(self)
        vbox = QVBoxLayout()
        self.canvas = MyMplCanvas(self, width=10, height=8, dpi=100)
        vbox.addWidget(self.canvas)
        hbox = QHBoxLayout()
        self.start_button = QPushButton("시작", self)
        self.stop_button = QPushButton("정지", self)
        self.start_button.clicked.connect(self.on_start)
        self.stop_button.clicked.connect(self.on_stop)
        hbox.addWidget(self.start_button)
        hbox.addWidget(self.stop_button)
        vbox.addLayout(hbox)
        self.setLayout(vbox)

        self.x = np.arange(60)
        self.y = np.ones(60, dtype=np.float) * np.nan
        self.line, = self.canvas.axes.plot(self.x, self.y, animated=True, lw=2)

        self.x2 = np.arange(60)
        self.y2 = np.ones(60, dtype=np.float) * np.nan
        self.line2, = self.canvas.axes2.plot(self.x2, self.y2, animated=True, color='red', lw=2)

    def update_line(self, i):
        y = int(temp_buf_list[0][2:])
        old_y = self.line.get_ydata()
        new_y = np.r_[old_y[1:], y]
        self.line.set_ydata(new_y)

        # self.line.set_ydata(y)
        return [self.line]

    def update_line2(self, i):
        y2 = int(temp_buf_list[1][2:])
        old_y2 = self.line2.get_ydata()
        new_y2 = np.r_[old_y2[1:], y2]
        self.line2.set_ydata(new_y2)
        return [self.line2]
        # self.line.set_ydata(y)

    def on_start(self):
        self.ani = animation.FuncAnimation(self.canvas.figure, self.update_line, blit=True, interval=25)
        self.ani2 = animation.FuncAnimation(self.canvas.figure, self.update_line2, blit=True, interval=25)

    def on_stop(self):
        self.ani._stop()
        self.ani2._stop()


class TimeCollector:
    def __init__(self):
        self.recordTime()

    def recordTime(self):
        time = QTime.currentTime()
        long_time = time.toString("[hh:mm:ss]\n")
        return long_time

class MsgBox(QWidget):
    def __init__(self, msg_str="Error", warning_flag=0):
        QWidget.__init__(self, flags=Qt.Widget)
        self.title = 'Error'
        self.left = 300
        self.top = 300
        self.width = 300
        self.height = 300
        self._msg_str = msg_str
        self._warning_flag = warning_flag
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.move(1000, 500)

        if self._warning_flag == 1:
            buttonReply = QMessageBox.warning(\
                self, 'Warning', self._msg_str, \
                QMessageBox.Ok)
        else:
            buttonReply = QMessageBox.information(\
                self, 'Info', self._msg_str, \
                QMessageBox.Ok)


class CLineEditWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUI()

    def setupUI(self):
        self.setWindowTitle("다른 이름으로 저장")
        self.setGeometry(600, 600, 300, 100)

        self.label = QLabel("이름 : ", self)
        self.label.move(20, 20)
        self.label.resize(150, 20)

        self.lineEdit = QLineEdit(".txt", self)
        self.lineEdit.move(60, 20)
        self.lineEdit.resize(200, 20)
        self.lineEdit.textChanged.connect(self.lineEdit_textChanged)

        self.statusBar = QStatusBar(self)
        self.setStatusBar(self.statusBar)

        btnSave = QPushButton("저장", self)
        btnSave.move(10, 50)
        btnSave.clicked.connect(self.btnSave_clicked)

        btnQuit = QPushButton("닫기", self)
        btnQuit.move(190,50)
        btnQuit.clicked.connect(self.btnQuit_clicked)

    def btnSave_clicked(self):
        print(self.lineEdit.text())
        msg = "저장하시겠습니까?"
        msg += "\n이름 : " + self.lineEdit.text()
        buttonReply = QMessageBox.question(self, '저장', msg, \
                                           QMessageBox.Yes | \
                                           QMessageBox.No, QMessageBox.No)

        if buttonReply == QMessageBox.Yes:
            os.rename('acc_log.txt', self.lineEdit.text())
            QMessageBox.about(self, "저장", "저장 되었습니다")
            self.statusBar.showMessage("저장 되었습니다")
            self.close()

        if buttonReply == QMessageBox.No:
            print('No clicked')
            os.remove('acc_log.txt')
            self.close()

    def btnQuit_clicked(self):
        os.remove('acc_log.txt')
        self.close()

    def lineEdit_textChanged(self):
        pass


__author__ = "Deokyu Lim <hong18s@gmail.com>"
__2nd_author__ = "Gwangrok Baek <100@ceedup.com>"
__platform__ = sys.platform


class SerialReadThread(QThread):
    """
    시리얼 연결이 성공하면 항상 데이터를 수신할 수 있어야 하므로
    스레드로 만들어야 한다.
    """
    # 사용자 정의 시그널 선언
    # 받은 데이터 그대로를 전달 해주기 위해 QByteArray 형태로 전달
    received_data = pyqtSignal(QByteArray, name="receivedData")

    def __init__(self, serial):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self._status = False
        self.mutex = QMutex()
        self.serial = serial

    def __del__(self):
        self.wait()

    def run(self):
        """
        들어온 데이터가 있다면 시그널을 발생
        :return:
        """
        global buf_list
        global temp_buf_list
        global done

        temp_str = ""

        while True:
            self.mutex.lock()
            if not self._status:
                self.cond.wait(self.mutex)

            buf = self.serial.readAll()

            if buf:
                self.received_data.emit(buf)

            self.usleep(1)
            self.mutex.unlock()

    def toggle_status(self):
        self._status = not self._status
        if self._status:
            self.cond.wakeAll()

    @pyqtSlot(bool, name='setStatus')
    def set_status(self, status):
        self._status = status
        if self._status:
            self.cond.wakeAll()


class SerialController(QWidget):
    # 시리얼포트 상수 값
    BAUDRATES = (
        QSerialPort.Baud1200,
        QSerialPort.Baud2400,
        QSerialPort.Baud4800,
        QSerialPort.Baud9600,
        QSerialPort.Baud19200,
        QSerialPort.Baud38400,
        QSerialPort.Baud57600,
        QSerialPort.Baud115200,
    )

    DATABITS = (
        QSerialPort.Data5,
        QSerialPort.Data6,
        QSerialPort.Data7,
        QSerialPort.Data8,
    )

    FLOWCONTROL = (
        QSerialPort.NoFlowControl,
        QSerialPort.HardwareControl,
        QSerialPort.SoftwareControl,
    )

    PARITY = (
        QSerialPort.NoParity,
        QSerialPort.EvenParity,
        QSerialPort.OddParity,
        QSerialPort.SpaceParity,
        QSerialPort.MarkParity,
    )

    STOPBITS = (
        QSerialPort.OneStop,
        QSerialPort.OneAndHalfStop,
        QSerialPort.TwoStop,

    )

    received_data = pyqtSignal(QByteArray, name="receivedData")
    sent_data = pyqtSignal(str, name="sentData")
    widget_thread = QThread()

    def __init__(self):
        QWidget.__init__(self, flags=Qt.Widget)
        # 위젯 선언
        self.gb = QGroupBox(self.tr("Serial"))
        self.cb_port = QComboBox()
        self.cb_baud_rate = QComboBox()
        self.cb_data_bits = QComboBox()
        self.cb_flow_control = QComboBox()
        self.cb_parity = QComboBox()
        self.cb_stop_bits = QComboBox()

        # 시리얼 인스턴스 생성
        # 시리얼 스레드 설정 및 시작
        self.serial = QSerialPort()
        self.serial_info = QSerialPortInfo()
        self.serial_read_thread = SerialReadThread(self.serial)
        self.serial_read_thread.received_data.connect(lambda v: self.received_data.emit(v))
        self.serial_read_thread.start()

        self.init_widget()

    def init_widget(self):
        self.setWindowTitle("Serial Controller")
        layout = QBoxLayout(QBoxLayout.TopToBottom, parent=self)
        grid_box = QGridLayout()

        grid_box.addWidget(QLabel(self.tr("Port")), 0, 0)
        grid_box.addWidget(self.cb_port, 0, 1)

        grid_box.addWidget(QLabel(self.tr("Baud Rate")), 1, 0)
        grid_box.addWidget(self.cb_baud_rate, 1, 1)

        grid_box.addWidget(QLabel(self.tr("Data Bits")), 2, 0)
        grid_box.addWidget(self.cb_data_bits, 2, 1)

        grid_box.addWidget(QLabel(self.tr("Flow Control")), 3, 0)
        grid_box.addWidget(self.cb_flow_control, 3, 1)

        grid_box.addWidget(QLabel(self.tr("Parity")), 4, 0)
        grid_box.addWidget(self.cb_parity, 4, 1)

        grid_box.addWidget(QLabel(self.tr("Stop Bits")), 5, 0)
        grid_box.addWidget(self.cb_stop_bits, 5, 1)

        self._fill_serial_info()
        self.gb.setLayout(grid_box)
        layout.addWidget(self.gb)
        self.setLayout(layout)

    def _fill_serial_info(self):
        # 시리얼 상수 값들을 위젯에 채운다
        self.cb_port.insertItems(0, self._get_available_port())
        self.cb_baud_rate.insertItems(0, [str(x) for x in self.BAUDRATES])
        self.cb_data_bits.insertItems(0, [str(x) for x in self.DATABITS])
        flow_name = {0: "None", 1: "Hardware", 2: "Software"}
        self.cb_flow_control.insertItems(0, [flow_name[x] for x in self.FLOWCONTROL])
        parity_name = {0: "None", 2: "Even", 3: "Odd", 4: "Space", 5: "Mark"}
        self.cb_parity.insertItems(0, [parity_name[x] for x in self.PARITY])
        stop_bits_name = {1: "1", 3: "1.5", 2: "2"}
        self.cb_stop_bits.insertItems(0, [stop_bits_name[x] for x in self.STOPBITS])

    @staticmethod
    def get_port_path():
        """
        현재플래폼에 맞게 경로 또는 지정어를 반환
        :return:
        """
        return {"linux": '/dev/ttyS', "win32": 'COM'}[__platform__]

    def _get_available_port(self):
        """
        255개의 포트를 열고 닫으면서 사용가능한 포트를 찾아서 반환
        :return:
        """
        available_port = list()
        port_path = self.get_port_path()

        for number in range(255):
            port_name = port_path + str(number)
            if not self._open(port_name):
                continue
            available_port.append(port_name)
            self.serial.close()
        return available_port

    def _open(self, port_name, baudrate=QSerialPort.Baud9600, data_bits=QSerialPort.Data8,
              flow_control=QSerialPort.NoFlowControl, parity=QSerialPort.NoParity, stop_bits=QSerialPort.OneStop):
        """
        인자값으로 받은 시리얼 접속 정보를 이용하여 해당 포트를 연결한다.
        :param port_name:
        :param baudrate:
        :param data_bits:
        :param flow_control:
        :param parity:
        :param stop_bits:
        :return: bool
        """
        info = QSerialPortInfo(port_name)
        self.serial.setPort(info)
        self.serial.setBaudRate(baudrate)
        self.serial.setDataBits(data_bits)
        self.serial.setFlowControl(flow_control)
        self.serial.setParity(parity)
        self.serial.setStopBits(stop_bits)
        return self.serial.open(QIODevice.ReadWrite)

    def connect_serial(self):
        serial_info = {
            "port_name": self.cb_port.currentText(),
            "baudrate": self.BAUDRATES[self.cb_baud_rate.currentIndex()],
            "data_bits": self.DATABITS[self.cb_data_bits.currentIndex()],
            "flow_control": self.FLOWCONTROL[self.cb_flow_control.currentIndex()],
            "parity": self.PARITY[self.cb_parity.currentIndex()],
            "stop_bits": self.STOPBITS[self.cb_stop_bits.currentIndex()],
        }
        status = self._open(**serial_info)
        self.serial_read_thread.setStatus(status)
        return status

    def disconnect_serial(self):
        return self.serial.close()

    @pyqtSlot(bytes, name="writeData")
    def write_data(self, data):
        self.serial.writeData(data)


class Form(QWidget):
    """
    테스트용도의 단독 실행때만 사용하는 폼
    """
    def __init__(self):
        QWidget.__init__(self, flags=Qt.Widget)
        self.te = QTextEdit()
        self.gp = AnimationWidget()
        self.pb = QPushButton("연결")
        self.pb_send = QPushButton("전송")
        self.serial = SerialController()
        self.pb_record = QPushButton("녹화")
        self.pb_store = QPushButton("데이터 저장")
        self.init_widget()
        self.record_status = False
        self.data_written = False
        self.write_file = False
        self.my_file = open('acc_log.txt', 'w')
        self.my_file.close()
        self.my_time = TimeCollector()
        self.store_window = CLineEditWindow()

        self.text_doc = QTextDocument()
        self.text_block = QTextBlock()
        self.temp_str = ""

    def init_widget(self):
        """
        현재 위젯의 모양등을 초기화
        """
        self.setWindowTitle("UART 데이터 녹화 프로그램")
        form_lbx = QBoxLayout(QBoxLayout.TopToBottom, parent=self)
        self.setLayout(form_lbx)

        self.pb.clicked.connect(self.slot_clicked_connect_button)
        self.serial.received_data.connect(self.read_data)
        #test_data = bytes([0x02]) + bytes("TEST DATA", "utf-8") + bytes([0x03])
        #self.pb_send.clicked.connect(lambda: self.serial.writeData(test_data))
        self.pb_record.clicked.connect(self.slot_clicked_record_button)
        self.pb_store.clicked.connect(self.slot_clicked_store_button)
        form_lbx.addWidget(self.serial)
        form_lbx.addWidget(self.te)
        form_lbx.addWidget(self.gp)
        form_lbx.addWidget(self.pb_send)
        form_lbx.addWidget(self.pb)
        form_lbx.addWidget(self.pb_record)
        form_lbx.addWidget(self.pb_store)

        # 많이 사용하는 옵션을 미리 지정해 둔다.
        # 9600 8N1
        self.serial.cb_baud_rate.setCurrentIndex(7)
        self.serial.cb_data_bits.setCurrentIndex(3)

    @pyqtSlot(QByteArray, name="readData")
    def read_data(self, rd):
        global temp_buf_list

        prev_cursor = self.te.textCursor()
        self.te.moveCursor(QTextCursor.End)
        acc_data = str(rd, 'ascii', 'replace')
        self.te.insertPlainText(acc_data)
        self.te.setTextCursor(prev_cursor)
        if self.write_file is True:
            self.my_file.write(acc_data)
        else:
            pass
        line_number = prev_cursor.blockNumber()
        self.text_doc = self.te.document()
        self.text_block = self.text_doc.findBlockByLineNumber(line_number - 1)
        self.temp_str = self.text_block.text()
        temp_buf_list = self.temp_str.split(" ")

        #if line_number > 50 & len(acc_data) > 23:
        #    self.te.clear()

    @pyqtSlot(name="clickedConnectButton")
    def slot_clicked_connect_button(self):
        if self.serial.serial.isOpen():
            self.serial.disconnect_serial()
        else:
            self.te.clear()
            self.serial.connect_serial()
        self.pb.setText({False: '연결', True: '연결 해제'}[self.serial.serial.isOpen()])

    @pyqtSlot(name="clickedRecordButton")
    def slot_clicked_record_button(self):
        if not self.serial.serial.isOpen():
            MsgBox("연결 후 녹화를 시작해주세요", 1)
            return
        if self.record_status:
            self.record_status = False
            self.write_file = False
            self.my_file.write(self.my_time.recordTime())
            self.my_file.close()
        else:
            self.record_status = True
            self.data_written = True
            self.write_file = True
            self.my_file = open('acc_log.txt', 'w')
            self.my_file.write('\n')
            self.my_file.write(self.my_time.recordTime())

        self.pb_record.setText({False: '녹화', True: '녹화 종료'}[self.record_status])

    @pyqtSlot(name="clickedStoreButton")
    def slot_clicked_store_button(self):
        if not self.data_written:
            MsgBox("저장가능한 데이터가 없습니다", 1)
            return
        else:
            self.record_status = False
            self.pb_record.setText('녹화')
            self.data_written = False
            if self.write_file is True:
                self.write_file = False
                self.my_file.write(self.my_time.recordTime())
                self.my_file.close()
            self.store_window.show()


if __name__ == "__main__":
    from PyQt5.QtWidgets import QPushButton
    from PyQt5.QtWidgets import QTextEdit
    app = QApplication(sys.argv)
    excepthook = sys.excepthook
    sys.excepthook = lambda t, val, tb: excepthook(t, val, tb)
    form = Form()
    form.show()
    exit(app.exec_())