import errno
import fcntl
import os
import pty
import signal
import socket
import sys
import time
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QSocketNotifier, Qt, QTimer
from PyQt5.QtGui import QFont, QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)


SERVER_HOST = "127.0.0.1"
SERVER_PORT = 9999
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BACKEND_WORKDIR = os.path.join(BASE_DIR, "build")
BACKEND_EXECUTABLE = os.path.join(BACKEND_WORKDIR, "bin", "motor_test")
PLOT_INTERVAL_MS = 100
RX_INTERVAL_MS = 20
DEFAULT_TEST_POS_DEG = 360.0
DEFAULT_TEST_VEL_DEG_S = 36.0
DEFAULT_LOAD_TORQUE_NM = 2.0
ZERO_GUARD_DEG = 1.0
TERMINAL_MAX_BLOCKS = 500
CAN_LOG_JOIN_WINDOW_SEC = 3.0
TELEMETRY_KEYS = {"TORQUE1", "POS1", "VEL1", "TORQUE2", "POS2", "VEL2"}

MOTOR_PRESETS = (
    ("50 标准", 40.0, 5.0, 0.45),
    ("50 加长", 40.0, 5.0, 0.45),
    ("70 标准", 80.0, 10.0, 0.35),
    ("85 标准", 160.0, 25.0, 0.50),
)


@dataclass
class MotorSample:
    torque: float = 0.0
    position: float = 0.0
    velocity: float = 0.0


class MotorControlApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.sock = None
        self.is_connected = False
        self.rx_buffer = b""
        self.test_token = 0
        self.last_can_log_id = None
        self.last_can_log_time = 0.0
        self.last_can_log_open = False
        self.backend_pid = None
        self.backend_fd = None
        self.backend_notifier = None

        self.motor_1 = MotorSample()
        self.motor_2 = MotorSample()
        self.target_pos = 0.0
        self.target_vel = 0.0
        self.peak_torque = 0.0
        self.actual_torque = 0.0

        self.arm_length_m = 0.35
        self.load_mass_kg = 10.0
        self.arm_mass_kg = 3.0
        self.gravity = 9.81

        self.data_length = 260
        self.time_axis = np.linspace(-26.0, 0.0, self.data_length)
        self.history = {
            "torque1": np.zeros(self.data_length),
            "torque2": np.zeros(self.data_length),
            "actual_torque": np.zeros(self.data_length),
            "pos1": np.zeros(self.data_length),
            "pos2": np.zeros(self.data_length),
            "target_pos": np.zeros(self.data_length),
            "vel1": np.zeros(self.data_length),
            "vel2": np.zeros(self.data_length),
            "target_vel": np.zeros(self.data_length),
        }

        self.current_preset_button = None
        self.value_labels = {}

        self.init_ui()
        self.update_connection_ui()
        self.refresh_test_estimate()

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.update_data)
        self.plot_timer.start(PLOT_INTERVAL_MS)

        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self.receive_data)

    def init_ui(self):
        self.setWindowTitle("双电机对拖测试台")
        self.setGeometry(100, 100, 1360, 860)
        self.setStyleSheet("""
            QWidget {
                background-color: #16181c;
                color: #eef1f5;
                font-size: 13px;
            }
            QGroupBox {
                border: 1px solid #323844;
                border-radius: 6px;
                margin-top: 12px;
                padding: 12px 10px 10px 10px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: #d9dee7;
            }
            QPushButton {
                background-color: #2d3542;
                border: 1px solid #3d4655;
                border-radius: 5px;
                min-height: 30px;
                padding: 6px 10px;
            }
            QPushButton:hover {
                background-color: #3a4454;
            }
            QPushButton:checked {
                background-color: #2f6fed;
                border-color: #75a3ff;
            }
            QPushButton:disabled {
                background-color: #242932;
                color: #6f7785;
            }
            QLabel[role="value"] {
                color: #dfe7f3;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 15px;
                padding: 4px 0;
            }
            QLabel[role="muted"] {
                color: #98a2b3;
            }
            QDoubleSpinBox {
                background-color: #101318;
                border: 1px solid #3d4655;
                border-radius: 4px;
                min-height: 28px;
                padding: 2px 6px;
            }
            QLineEdit, QPlainTextEdit {
                background-color: #0d1117;
                border: 1px solid #303846;
                border-radius: 4px;
                color: #dfe7f3;
                font-family: "Consolas", "Courier New", monospace;
            }
            QLineEdit {
                min-height: 28px;
                padding: 2px 6px;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        main_layout.addWidget(self.create_control_panel(), 0)
        main_layout.addWidget(self.create_plot_panel(), 1)

    def create_control_panel(self):
        panel = QWidget()
        panel.setFixedWidth(370)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        nav_layout = QHBoxLayout()
        self.btn_nav_overview = self.make_nav_button("运行")
        self.btn_nav_debug = self.make_nav_button("调试")
        self.btn_nav_overview.setChecked(True)
        self.btn_nav_overview.clicked.connect(lambda: self.switch_panel(0))
        self.btn_nav_debug.clicked.connect(lambda: self.switch_panel(1))
        nav_layout.addWidget(self.btn_nav_overview)
        nav_layout.addWidget(self.btn_nav_debug)
        layout.addLayout(nav_layout)

        self.control_stack = QStackedWidget()
        self.control_stack.addWidget(self.create_overview_page())
        self.control_stack.addWidget(self.create_debug_page())
        layout.addWidget(self.control_stack, 1)
        layout.addWidget(self.create_terminal_group(), 0)
        return panel

    def make_nav_button(self, text):
        button = QPushButton(text)
        button.setCheckable(True)
        button.setCursor(Qt.PointingHandCursor)
        return button

    def create_overview_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        layout.addWidget(self.create_connection_group())
        layout.addWidget(self.create_realtime_group())
        layout.addWidget(self.create_stats_group())
        layout.addStretch()
        return page

    def create_debug_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        layout.addWidget(self.create_preset_group())
        layout.addWidget(self.create_test_group())
        layout.addWidget(self.create_command_group())
        layout.addStretch()
        return page

    def create_connection_group(self):
        group = QGroupBox("系统连接")
        layout = QVBoxLayout(group)

        self.btn_connect = QPushButton("连接服务器")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.lbl_status = QLabel()
        self.lbl_status.setProperty("role", "value")
        self.lbl_endpoint = QLabel(f"{SERVER_HOST}:{SERVER_PORT}")
        self.lbl_endpoint.setProperty("role", "muted")

        layout.addWidget(self.btn_connect)
        layout.addWidget(self.lbl_status)
        layout.addWidget(self.lbl_endpoint)
        return group

    def create_realtime_group(self):
        group = QGroupBox("实时数据")
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)

        headers = ("电机", "扭矩", "位置", "速度")
        for col, text in enumerate(headers):
            label = QLabel(text)
            label.setProperty("role", "muted")
            grid.addWidget(label, 0, col)

        self.add_motor_row(grid, 1, "motor1", "电机 1")
        self.add_motor_row(grid, 2, "motor2", "电机 2")
        return group

    def add_motor_row(self, grid, row, key, title):
        grid.addWidget(QLabel(title), row, 0)
        for col, metric in enumerate(("torque", "position", "velocity"), start=1):
            label = QLabel("0.00")
            label.setProperty("role", "value")
            self.value_labels[f"{key}_{metric}"] = label
            grid.addWidget(label, row, col)

    def create_model_group(self):
        group = QGroupBox("模型参数")
        layout = QGridLayout(group)

        self.spin_weight = self.make_spinbox(0.0, 50.0, self.load_mass_kg, 0.5, " kg")
        self.spin_arm = self.make_spinbox(0.1, 1.5, self.arm_length_m, 0.05, " m")
        self.spin_weight.valueChanged.connect(self.update_phys_params)
        self.spin_arm.valueChanged.connect(self.update_phys_params)

        layout.addWidget(QLabel("负载质量"), 0, 0)
        layout.addWidget(self.spin_weight, 0, 1)
        layout.addWidget(QLabel("力臂长度"), 1, 0)
        layout.addWidget(self.spin_arm, 1, 1)
        return group

    def create_stats_group(self):
        group = QGroupBox("统计")
        layout = QGridLayout(group)

        self.lbl_peak_torque = QLabel("0.00 Nm")
        self.lbl_peak_torque.setProperty("role", "value")
        self.lbl_actual_torque = QLabel("0.00 Nm")
        self.lbl_actual_torque.setProperty("role", "value")
        self.btn_clear_peak = QPushButton("清除峰值")
        self.btn_clear_peak.clicked.connect(self.reset_peak_torque)

        layout.addWidget(QLabel("反馈峰值"), 0, 0)
        layout.addWidget(self.lbl_peak_torque, 0, 1)
        layout.addWidget(QLabel("模型扭矩"), 1, 0)
        layout.addWidget(self.lbl_actual_torque, 1, 1)
        layout.addWidget(self.btn_clear_peak, 2, 0, 1, 2)
        return group

    def create_preset_group(self):
        group = QGroupBox("电机选型")
        grid = QGridLayout(group)
        self.preset_buttons = []

        for index, (name, max_torque, mass, arm) in enumerate(MOTOR_PRESETS):
            button = QPushButton(f"{name}\n{max_torque:.0f} Nm")
            button.setCheckable(True)
            button.clicked.connect(
                lambda _, b=button, p=(max_torque, mass, arm): self.apply_motor_preset(b, p)
            )
            grid.addWidget(button, index // 2, index % 2)
            self.preset_buttons.append(button)

        return group

    def create_test_group(self):
        group = QGroupBox("对拖测试")
        layout = QGridLayout(group)

        self.spin_test_pos = self.make_spinbox(-360.0, 360.0, DEFAULT_TEST_POS_DEG, 10.0, " deg")
        self.spin_test_vel = self.make_spinbox(1.0, 180.0, DEFAULT_TEST_VEL_DEG_S, 1.0, " deg/s")
        self.spin_drag_load = self.make_spinbox(0.0, 40.0, DEFAULT_LOAD_TORQUE_NM, 0.5, " Nm")
        self.lbl_test_estimate = QLabel()
        self.lbl_test_estimate.setProperty("role", "muted")

        for spin in (self.spin_test_pos, self.spin_test_vel, self.spin_drag_load):
            spin.valueChanged.connect(self.refresh_test_estimate)

        layout.addWidget(QLabel("相对角度"), 0, 0)
        layout.addWidget(self.spin_test_pos, 0, 1)
        layout.addWidget(QLabel("目标速度"), 1, 0)
        layout.addWidget(self.spin_test_vel, 1, 1)
        layout.addWidget(QLabel("负载扭矩"), 2, 0)
        layout.addWidget(self.spin_drag_load, 2, 1)
        layout.addWidget(self.lbl_test_estimate, 3, 0, 1, 2)

        self.btn_test = QPushButton("启动往返测试")
        self.btn_test.clicked.connect(self.run_test)
        layout.addWidget(self.btn_test, 4, 0, 1, 2)
        return group

    def create_command_group(self):
        group = QGroupBox("手动命令")
        layout = QGridLayout(group)

        self.btn_zero = QPushButton("置零")
        self.btn_zero.clicked.connect(self.set_zero_position)
        self.btn_disable = QPushButton("失能")
        self.btn_disable.clicked.connect(self.motor_disable)
        self.btn_forward = QPushButton("正向")
        self.btn_forward.clicked.connect(lambda: self.move_once(1.0))
        self.btn_reverse = QPushButton("反向")
        self.btn_reverse.clicked.connect(lambda: self.move_once(-1.0))

        layout.addWidget(self.btn_zero, 0, 0)
        layout.addWidget(self.btn_disable, 0, 1)
        layout.addWidget(self.btn_forward, 1, 0)
        layout.addWidget(self.btn_reverse, 1, 1)
        return group

    def create_terminal_group(self):
        group = QGroupBox("后端终端")
        layout = QVBoxLayout(group)

        backend_layout = QHBoxLayout()
        self.btn_backend_start = QPushButton("启动后端")
        self.btn_backend_start.clicked.connect(self.start_backend_terminal)
        self.btn_backend_stop = QPushButton("停止后端")
        self.btn_backend_stop.clicked.connect(self.stop_backend_terminal)
        self.lbl_backend_status = QLabel("后端: 未启动")
        self.lbl_backend_status.setProperty("role", "muted")

        backend_layout.addWidget(self.btn_backend_start)
        backend_layout.addWidget(self.btn_backend_stop)
        backend_layout.addWidget(self.lbl_backend_status, 1)

        self.terminal_output = QPlainTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setMinimumHeight(180)
        self.terminal_output.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        self.terminal_output.document().setMaximumBlockCount(TERMINAL_MAX_BLOCKS)

        input_layout = QHBoxLayout()
        self.terminal_input = QLineEdit()
        self.terminal_input.setPlaceholderText("输入后端命令后回车，例如: p_info")
        self.terminal_input.returnPressed.connect(self.send_terminal_command)
        self.btn_terminal_send = QPushButton("回车")
        self.btn_terminal_send.clicked.connect(self.send_terminal_command)
        self.btn_terminal_clear = QPushButton("清空")
        self.btn_terminal_clear.clicked.connect(self.clear_terminal)

        input_layout.addWidget(self.terminal_input, 1)
        input_layout.addWidget(self.btn_terminal_send)
        input_layout.addWidget(self.btn_terminal_clear)

        layout.addLayout(backend_layout)
        layout.addWidget(self.terminal_output)
        layout.addLayout(input_layout)
        return group

    def make_spinbox(self, minimum, maximum, value, step, suffix=""):
        spinbox = QDoubleSpinBox()
        spinbox.setRange(minimum, maximum)
        spinbox.setDecimals(2)
        spinbox.setSingleStep(step)
        spinbox.setValue(value)
        spinbox.setSuffix(suffix)
        return spinbox

    def create_plot_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        self.plot_torque = self.make_plot("扭矩反馈", "N·m")
        self.curve_torque1 = self.plot_torque.plot(pen=pg.mkPen("#f5c542", width=2), name="电机 1")
        self.curve_torque2 = self.plot_torque.plot(pen=pg.mkPen("#9b7bff", width=2), name="电机 2")
        self.curve_actual_torque = self.plot_torque.plot(
            pen=pg.mkPen("#4fc3f7", width=1, style=Qt.DashLine),
            name="模型",
        )

        self.plot_pos = self.make_plot("位置", "deg")
        self.curve_pos1 = self.plot_pos.plot(pen=pg.mkPen("#52d273", width=2), name="电机 1")
        self.curve_pos2 = self.plot_pos.plot(pen=pg.mkPen("#48c6d9", width=2), name="电机 2")
        self.curve_target_pos = self.plot_pos.plot(
            pen=pg.mkPen("#f58f42", width=1, style=Qt.DashLine),
            name="目标",
        )

        self.plot_vel = self.make_plot("速度", "deg/s")
        self.curve_vel1 = self.plot_vel.plot(pen=pg.mkPen("#ff6b9a", width=2), name="电机 1")
        self.curve_vel2 = self.plot_vel.plot(pen=pg.mkPen("#75d377", width=2), name="电机 2")
        self.curve_target_vel = self.plot_vel.plot(
            pen=pg.mkPen("#57b8ff", width=1, style=Qt.DashLine),
            name="目标",
        )

        for plot in (self.plot_torque, self.plot_pos, self.plot_vel):
            layout.addWidget(plot)

        return panel

    def make_plot(self, title, unit):
        plot = pg.PlotWidget(title=f"{title} ({unit})")
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.addLegend(offset=(10, 10))
        plot.setBackground("#101318")
        plot.setLabel("bottom", "时间", units="s")
        return plot

    def switch_panel(self, index):
        self.control_stack.setCurrentIndex(index)
        self.btn_nav_overview.setChecked(index == 0)
        self.btn_nav_debug.setChecked(index == 1)

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_all()
            return

        self.connect_server()

    def connect_server(self):
        if self.is_connected:
            return

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2)
            self.sock.connect((SERVER_HOST, SERVER_PORT))
            self.sock.setblocking(False)
            self.is_connected = True
            self.rx_timer.start(RX_INTERVAL_MS)
            self.append_terminal(f"! 已连接 {SERVER_HOST}:{SERVER_PORT}")
        except OSError as exc:
            self.disconnect_all()
            self.append_terminal(f"! 连接失败: {exc}")
            self.lbl_status.setText(f"连接失败: {str(exc)[:28]}")
        finally:
            self.update_connection_ui()

    def disconnect_all(self):
        self.is_connected = False
        self.rx_buffer = b""
        self.test_token += 1
        self.rx_timer.stop()

        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

        self.update_connection_ui()

    def backend_running(self):
        if self.backend_pid is None:
            return False

        try:
            pid, _ = os.waitpid(self.backend_pid, os.WNOHANG)
        except ChildProcessError:
            self.finish_backend_terminal()
            return False

        if pid == 0:
            return True

        self.finish_backend_terminal()
        return False

    def start_backend_terminal(self):
        if self.backend_running():
            self.append_terminal("! 后端已经在运行")
            return

        if not os.path.exists(BACKEND_EXECUTABLE):
            self.append_terminal(f"! 未找到后端程序: {BACKEND_EXECUTABLE}")
            return
        if not os.access(BACKEND_EXECUTABLE, os.X_OK):
            self.append_terminal(f"! 后端程序没有执行权限: {BACKEND_EXECUTABLE}")
            return

        try:
            pid, fd = pty.fork()
        except OSError as exc:
            self.append_terminal(f"! 启动后端失败: {exc}")
            return

        if pid == 0:
            try:
                os.chdir(BACKEND_WORKDIR)
                os.execv(BACKEND_EXECUTABLE, [BACKEND_EXECUTABLE])
            except OSError as exc:
                os.write(2, f"exec motor_test failed: {exc}\n".encode("utf-8"))
                os._exit(127)

        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        self.backend_pid = pid
        self.backend_fd = fd
        self.backend_notifier = QSocketNotifier(fd, QSocketNotifier.Read, self)
        self.backend_notifier.activated.connect(self.read_backend_terminal)
        self.append_terminal(f"! 后端已启动: {BACKEND_EXECUTABLE}")
        self.update_backend_ui()

    def stop_backend_terminal(self):
        if not self.backend_running():
            self.finish_backend_terminal()
            return

        try:
            os.kill(self.backend_pid, signal.SIGTERM)
        except OSError:
            pass

        QTimer.singleShot(300, self.force_stop_backend_terminal)
        if self.is_connected:
            self.disconnect_all()

    def force_stop_backend_terminal(self):
        if self.backend_pid is None:
            return
        pid = self.backend_pid
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
        try:
            os.waitpid(pid, 0)
        except ChildProcessError:
            pass
        self.backend_pid = None
        self.finish_backend_terminal()

    def finish_backend_terminal(self):
        if self.backend_notifier:
            self.backend_notifier.setEnabled(False)
            self.backend_notifier.deleteLater()
            self.backend_notifier = None

        if self.backend_fd is not None:
            try:
                os.close(self.backend_fd)
            except OSError:
                pass
            self.backend_fd = None

        if self.backend_pid is not None:
            try:
                os.waitpid(self.backend_pid, os.WNOHANG)
            except ChildProcessError:
                pass
            self.backend_pid = None

        self.update_backend_ui()

    def update_backend_ui(self):
        running = self.backend_pid is not None
        if hasattr(self, "lbl_backend_status"):
            self.lbl_backend_status.setText("后端: 运行中" if running else "后端: 未启动")
            self.lbl_backend_status.setStyleSheet(
                "color: #58d68d;" if running else "color: #98a2b3;"
            )
        if hasattr(self, "btn_backend_start"):
            self.btn_backend_start.setEnabled(not running)
        if hasattr(self, "btn_backend_stop"):
            self.btn_backend_stop.setEnabled(running)
        if hasattr(self, "terminal_input"):
            self.terminal_input.setEnabled(running or self.is_connected)
        if hasattr(self, "btn_terminal_send"):
            self.btn_terminal_send.setEnabled(running or self.is_connected)

    def read_backend_terminal(self):
        if self.backend_fd is None:
            return

        while True:
            try:
                data = os.read(self.backend_fd, 4096)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.EIO, errno.EBADF):
                    self.finish_backend_terminal()
                    return
                self.append_terminal(f"! 读取后端终端失败: {exc}")
                self.finish_backend_terminal()
                return

            if not data:
                self.finish_backend_terminal()
                return

            text = data.decode("utf-8", errors="replace")
            text = text.replace("\r\n", "\n").replace("\r", "\n")
            self.append_terminal_raw(text)
            if "TCP Server listening" in text and not self.is_connected:
                QTimer.singleShot(100, self.connect_server)

    def update_connection_ui(self):
        connected = self.is_connected and self.sock is not None
        self.lbl_status.setText("状态: 已连接" if connected else "状态: 未连接")
        self.lbl_status.setStyleSheet("color: #58d68d;" if connected else "color: #ff6b6b;")
        self.btn_connect.setText("断开连接" if connected else "连接服务器")
        self.btn_connect.setStyleSheet(
            "background-color: #9b2f35; border-color: #d96b72;"
            if connected
            else "background-color: #266f43; border-color: #58d68d;"
        )

        for button in (
            self.btn_test,
            self.btn_zero,
            self.btn_disable,
            self.btn_forward,
            self.btn_reverse,
            self.btn_clear_peak,
        ):
            button.setEnabled(connected)
        for button in getattr(self, "preset_buttons", []):
            button.setEnabled(connected)
        self.update_backend_ui()

    def reset_can_log_join(self):
        self.last_can_log_id = None
        self.last_can_log_time = 0.0
        self.last_can_log_open = False

    def scroll_terminal_to_bottom(self):
        scroll_bar = self.terminal_output.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def clear_terminal(self):
        if hasattr(self, "terminal_output"):
            self.terminal_output.clear()
        self.reset_can_log_join()

    def append_terminal(self, message, reset_can_join=True):
        if not hasattr(self, "terminal_output"):
            return
        self.terminal_output.appendPlainText(message)
        self.scroll_terminal_to_bottom()
        if reset_can_join:
            self.reset_can_log_join()

    def append_terminal_raw(self, text):
        if not hasattr(self, "terminal_output"):
            return
        cursor = self.terminal_output.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text)
        self.terminal_output.setTextCursor(cursor)
        self.scroll_terminal_to_bottom()
        self.reset_can_log_join()

    def append_terminal_inline(self, text):
        if not hasattr(self, "terminal_output"):
            return
        cursor = self.terminal_output.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text)
        self.terminal_output.setTextCursor(cursor)
        self.scroll_terminal_to_bottom()

    def append_backend_log(self, message):
        can_id, separator, payload = message.partition(":")
        is_can_fragment = can_id.startswith("[") and can_id.endswith("]") and separator
        if not is_can_fragment:
            self.append_terminal(f"< {message}")
            return

        self.append_can_log_fragment(can_id, payload)

    def append_can_log_fragment(self, can_id, payload, complete=False):
        now = time.monotonic()
        should_join = (
            self.last_can_log_open
            and self.last_can_log_id == can_id
            and now - self.last_can_log_time <= CAN_LOG_JOIN_WINDOW_SEC
        )

        if should_join:
            self.append_terminal_inline(payload)
        else:
            self.append_terminal(f"< {can_id}:{payload}", reset_can_join=False)

        self.last_can_log_id = can_id
        self.last_can_log_time = now
        self.last_can_log_open = not complete

    def append_can_packet_log(self, raw_line):
        parts = raw_line.split(" ", 3)
        if len(parts) < 4:
            self.append_terminal(f"< {raw_line}")
            return

        _, can_id, complete, payload = parts
        self.append_can_log_fragment(f"[{can_id}]", payload, complete == "1")

    def append_can_line_log(self, raw_line):
        parts = raw_line.split(" ", 2)
        if len(parts) < 3:
            self.append_terminal(f"< {raw_line}")
            return

        _, can_id, payload = parts
        self.append_terminal(f"< [{can_id}]:{payload}")

    def format_command_for_log(self, command):
        if isinstance(command, bytes):
            text = command.decode("utf-8", errors="ignore")
        else:
            text = str(command)
        return text.rstrip("\r\n")

    def send_terminal_command(self):
        command = self.terminal_input.text()
        self.terminal_input.clear()

        if self.backend_running() and self.backend_fd is not None:
            try:
                os.write(self.backend_fd, f"{command}\n".encode("utf-8"))
            except OSError as exc:
                self.append_terminal(f"! 后端输入失败: {exc}")
                self.finish_backend_terminal()
            return

        command = command.strip()
        if not command:
            return
        if not self.is_connected:
            self.append_terminal("! 后端未启动且 TCP 未连接，命令未发送")
            return

        self.send_command(f"{command}\n")

    def send_command(self, command, log_command=True):
        if not self.is_connected or not self.sock:
            return False

        try:
            data = command.encode("utf-8") if isinstance(command, str) else command
            if log_command:
                self.append_terminal(f"> {self.format_command_for_log(command)}")
            self.sock.sendall(data)
            return True
        except (BrokenPipeError, ConnectionResetError, OSError) as exc:
            message = f"发送失败: {exc}"
            print(message)
            self.append_terminal(f"! {message}")
            self.disconnect_all()
            return False

    def receive_data(self):
        if not self.is_connected or not self.sock:
            return

        try:
            while True:
                data = self.sock.recv(4096)
                if not data:
                    message = "服务器已断开连接"
                    print(message)
                    self.append_terminal(f"! {message}")
                    self.disconnect_all()
                    return
                self.rx_buffer += data
                lines = self.rx_buffer.split(b"\n")
                self.rx_buffer = lines.pop()
                self.parse_telemetry(
                    line.decode("utf-8", errors="replace") for line in lines
                )
        except BlockingIOError:
            return
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            message = f"接收失败: {exc}"
            print(message)
            self.append_terminal(f"! {message}")
            self.disconnect_all()
        except (UnicodeDecodeError, ValueError, IndexError):
            return

    def parse_telemetry(self, lines):
        for raw_line in lines:
            parts = raw_line.split()
            if not parts:
                continue
            key = parts[0].strip()
            if key == "POS_WITH_VEL_COMPLETE":
                self.target_vel = 0.0
                if self.backend_pid is None:
                    self.append_terminal("< POS_WITH_VEL_COMPLETE")
                continue
            if key == "LOG_CAN_LINE":
                if self.backend_pid is None:
                    self.append_can_line_log(raw_line)
                continue
            if key == "LOG_CAN":
                if self.backend_pid is None:
                    self.append_can_packet_log(raw_line)
                continue
            if key == "LOG":
                if self.backend_pid is None:
                    self.append_backend_log(raw_line[4:])
                continue
            if key not in TELEMETRY_KEYS:
                if self.backend_pid is None:
                    self.append_terminal(f"< {raw_line}")
                continue
            if len(parts) < 2:
                continue

            value = float(parts[1])
            if key == "TORQUE1":
                self.motor_1.torque = value
            elif key == "POS1":
                self.motor_1.position = value
            elif key == "VEL1":
                self.motor_1.velocity = value
            elif key == "TORQUE2":
                self.motor_2.torque = value
            elif key == "POS2":
                self.motor_2.position = value
            elif key == "VEL2":
                self.motor_2.velocity = value

        self.peak_torque = max(
            self.peak_torque,
            abs(self.motor_1.torque),
            abs(self.motor_2.torque),
        )

    def update_data(self):
        self.update_actual_torque()
        self.push_history()
        self.update_labels()
        self.update_plots()

    def update_actual_torque(self):
        theta_rad = np.deg2rad(self.motor_1.position)
        self.actual_torque = (
            self.load_mass_kg * self.arm_length_m + self.arm_mass_kg * 0.25
        ) * self.gravity * np.sin(theta_rad)

    def push_history(self):
        values = {
            "torque1": self.motor_1.torque,
            "torque2": self.motor_2.torque,
            "actual_torque": self.actual_torque,
            "pos1": self.motor_1.position,
            "pos2": self.motor_2.position,
            "target_pos": self.target_pos,
            "vel1": self.motor_1.velocity,
            "vel2": self.motor_2.velocity,
            "target_vel": self.target_vel,
        }
        for key, value in values.items():
            self.history[key] = np.roll(self.history[key], -1)
            self.history[key][-1] = value

    def update_labels(self):
        self.value_labels["motor1_torque"].setText(f"{self.motor_1.torque:.2f} Nm")
        self.value_labels["motor1_position"].setText(f"{self.motor_1.position:.2f} deg")
        self.value_labels["motor1_velocity"].setText(f"{self.motor_1.velocity:.2f} deg/s")
        self.value_labels["motor2_torque"].setText(f"{self.motor_2.torque:.2f} Nm")
        self.value_labels["motor2_position"].setText(f"{self.motor_2.position:.2f} deg")
        self.value_labels["motor2_velocity"].setText(f"{self.motor_2.velocity:.2f} deg/s")
        self.lbl_peak_torque.setText(f"{self.peak_torque:.2f} Nm")
        self.lbl_actual_torque.setText(f"{self.actual_torque:.2f} Nm")

    def update_plots(self):
        self.curve_torque1.setData(self.time_axis, self.history["torque1"])
        self.curve_torque2.setData(self.time_axis, self.history["torque2"])
        self.curve_actual_torque.setData(self.time_axis, self.history["actual_torque"])
        self.curve_pos1.setData(self.time_axis, self.history["pos1"])
        self.curve_pos2.setData(self.time_axis, self.history["pos2"])
        self.curve_target_pos.setData(self.time_axis, self.history["target_pos"])
        self.curve_vel1.setData(self.time_axis, self.history["vel1"])
        self.curve_vel2.setData(self.time_axis, self.history["vel2"])
        self.curve_target_vel.setData(self.time_axis, self.history["target_vel"])

    def apply_motor_preset(self, button, preset):
        max_torque, mass, arm = preset
        if self.current_preset_button:
            self.current_preset_button.setChecked(False)
        button.setChecked(True)
        self.current_preset_button = button

        if not self.send_command(f"SET_MAX_TORQUE {max_torque}\n"):
            return

        self.load_mass_kg = mass
        self.arm_length_m = arm
        if hasattr(self, "spin_weight"):
            self.spin_weight.setValue(mass)
        if hasattr(self, "spin_arm"):
            self.spin_arm.setValue(arm)

    def refresh_test_estimate(self):
        if not hasattr(self, "spin_test_pos"):
            return
        duration = self.single_move_duration_ms() / 1000.0
        self.lbl_test_estimate.setText(f"单程约 {duration:.1f} s，往返约 {duration * 2.0:.1f} s")

    def single_move_duration_ms(self):
        velocity = max(abs(self.spin_test_vel.value()), 0.001)
        distance = abs(self.spin_test_pos.value())
        return int(distance / velocity * 1000)

    def run_test(self):
        if not self.is_connected:
            return

        self.test_token += 1
        token = self.test_token
        move_ms = self.single_move_duration_ms()
        settle_ms = 500
        zero_delay_ms = 300

        self.reset_peak_torque()
        if not self.motor_disable():
            return
        if not self.set_zero_position():
            return

        QTimer.singleShot(zero_delay_ms, lambda: self.start_test_move(token, 1.0))
        QTimer.singleShot(
            zero_delay_ms + move_ms + settle_ms,
            lambda: self.run_if_current(token, lambda: self.move_once(-1.0)),
        )
        QTimer.singleShot(
            zero_delay_ms + move_ms * 2 + settle_ms * 2,
            lambda: self.run_if_current(token, self.motor_disable),
        )

    def start_test_move(self, token, direction):
        if token != self.test_token or not self.is_connected:
            return
        if abs(self.motor_1.position) > ZERO_GUARD_DEG:
            print("电机位置未置零，测试中止")
            self.motor_disable()
            return
        self.move_once(direction)

    def run_if_current(self, token, callback):
        if token == self.test_token and self.is_connected:
            callback()

    def move_once(self, direction):
        pos = abs(self.spin_test_pos.value()) * direction
        velocity = self.spin_test_vel.value()
        load_torque = self.spin_drag_load.value()
        return self.position_with_velocity(pos, velocity, load_torque)

    def position_with_velocity(self, pos, velocity, torque=0.0):
        if self.send_command(f"POS_WITH_VEL {pos:.2f} {torque:.2f} {velocity:.2f}\n"):
            self.target_pos += pos
            self.target_vel = abs(velocity) if pos >= 0.0 else -abs(velocity)
            print(f"Sent drag command: pos={pos:.2f}, torque={torque:.2f}, velocity={velocity:.2f}")
            return True
        return False

    def motor_disable(self):
        self.target_vel = 0.0
        return self.send_command(b"DISABLE_MOTOR\n")

    def set_zero_position(self):
        if self.send_command(b"ZERO\n"):
            self.target_pos = 0.0
            return True
        return False

    def reset_peak_torque(self):
        self.peak_torque = 0.0
        self.lbl_peak_torque.setText("0.00 Nm")

    def update_phys_params(self):
        if hasattr(self, "spin_weight"):
            self.load_mass_kg = self.spin_weight.value()
        if hasattr(self, "spin_arm"):
            self.arm_length_m = self.spin_arm.value()

    def closeEvent(self, event):
        self.disconnect_all()
        if self.backend_pid is not None:
            self.force_stop_backend_terminal()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 9))
    window = MotorControlApp()
    window.show()
    sys.exit(app.exec_())
