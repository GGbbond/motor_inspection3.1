import errno
import fcntl
import os
import pty
import re
import signal
import socket
import stat
import sys
import time
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QPoint, QProcess, QSize, QSocketNotifier, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QIcon, QPainter, QPen, QPixmap, QTextCursor
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
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
DEFAULT_DY200_DEVICE = "/dev/ttyUSB0"
DEFAULT_DY200_BAUD = 115200
ZERO_GUARD_DEG = 1.0
POSITION_CHECK_SETTLE_MS = 500
POSITION_CHECK_TOLERANCE_DEG = 1.0
TEST_SETTLE_MS = 500
TEST_ZERO_DELAY_MS = 300
PROGRESS_INTERVAL_MS = 100
TERMINAL_MAX_BLOCKS = 500
CAN_LOG_JOIN_WINDOW_SEC = 3.0
TELEMETRY_KEYS = {
    "TORQUE1",
    "POS1",
    "VEL1",
    "TEMP_MOS1",
    "TEMP_ROTOR1",
    "TORQUE2",
    "POS2",
    "VEL2",
    "TEMP_MOS2",
    "TEMP_ROTOR2",
    "SENSOR_TORQUE",
    "SENSOR_SPEED",
}

MOTOR_PRESETS = {
    "Bxi_motor_50": ("50 标准", 40.0, 5.0, 0.45),
    "Bxi_motor_50L": ("50 加长", 40.0, 5.0, 0.45),
    "Bxi_motor_70": ("70 标准", 80.0, 10.0, 0.35),
    "Bxi_motor_85": ("85 标准", 160.0, 25.0, 0.50),
}
MOTOR_PROGRAM_TO_MODEL = {
    "motor_50": "Bxi_motor_50",
    "motor_50_L": "Bxi_motor_50L",
    "motor_70": "Bxi_motor_70",
    "motor_85": "Bxi_motor_85",
}
MOTOR_PROGRAM_PATTERN = re.compile(
    r"\[(?P<source>\d+)\]:Program\s+for\s+"
    r"(?P<program>motor_50_L|motor_50|motor_70|motor_85)\b"
)
MOTOR_CAN_ID_PATTERN = re.compile(r"\[(?P<source>\d+)\]:Set CAN ID to (?P<can_id>\d+)")


@dataclass
class MotorSample:
    torque: float = 0.0
    position: float = 0.0
    velocity: float = 0.0
    temp_mos: float = 0.0
    temp_rotor: float = 0.0


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
        self.terminal_dialog = None
        self.terminal_dialog_output = None
        self.terminal_dialog_input = None
        self.terminal_dialog_send = None
        self.backend_parse_buffer = ""
        self.motor_source_can_ids = {}
        self.motor_models_by_source = {}
        self.detected_motor_model = None
        self.applied_motor_model = None
        self.pending_motor_model = None
        self.detected_motor_max_torque = 0.0
        self.motor_detect_status_labels = []
        self.motor_detect_model_labels = []
        self.motor_detect_torque_labels = []
        self.test_buttons = []
        self.test_progress_bars = []
        self.test_status_labels = []
        self.test_in_progress = False
        self.post_check_start_pos = 0.0
        self.progress_start_value = 0.0
        self.progress_end_value = 0.0
        self.progress_start_time = 0.0
        self.progress_duration_ms = 0
        self.progress_text = "待机"

        self.motor_1 = MotorSample()
        self.motor_2 = MotorSample()
        self.sensor_torque = np.nan
        self.sensor_speed = np.nan
        self.sensor_peak_torque = np.nan
        self.sensor_peak_speed = np.nan
        self.sensor_connected = False
        self.sensor_auto_connect_attempted = False
        self.sensor_connecting = False
        self.sensor_permission_process = None
        self.sensor_status_labels = []
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
            "sensor_torque": np.full(self.data_length, np.nan),
            "sensor_speed": np.full(self.data_length, np.nan),
        }

        self.value_labels = {}

        self.init_ui()
        self.update_connection_ui()
        self.refresh_test_estimate()

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.update_data)
        self.plot_timer.start(PLOT_INTERVAL_MS)

        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self.receive_data)

        self.progress_timer = QTimer(self)
        self.progress_timer.timeout.connect(self.update_test_progress_animation)

        QTimer.singleShot(100, self.start_backend_terminal)

    def init_ui(self):
        self.setWindowTitle("双电机对拖测试台")
        self.resize(1280, 740)
        self.setMinimumSize(1000, 600)
        self.setStyleSheet("""
            QWidget {
                background-color: #16181c;
                color: #eef1f5;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #323844;
                border-radius: 6px;
                margin-top: 10px;
                padding: 10px 8px 8px 8px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
                color: #d9dee7;
            }
            QPushButton {
                background-color: #2d3542;
                border: 1px solid #3d4655;
                border-radius: 5px;
                min-height: 34px;
                padding: 3px 8px;
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
            QLabel {
                min-height: 20px;
                padding: 1px 0;
            }
            QLabel[role="value"] {
                color: #dfe7f3;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 13px;
                min-height: 22px;
                padding: 2px 0;
            }
            QLabel[role="muted"] {
                color: #98a2b3;
            }
            QDoubleSpinBox {
                background-color: #101318;
                border: 1px solid #3d4655;
                border-radius: 4px;
                min-height: 34px;
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
                min-height: 34px;
                padding: 2px 6px;
            }
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QScrollArea > QWidget > QWidget {
                background-color: transparent;
            }
            QProgressBar {
                background-color: #101318;
                border: 1px solid #303846;
                border-radius: 4px;
                min-height: 22px;
                max-height: 22px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #2f6fed;
                border-radius: 3px;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        main_layout.addWidget(self.create_control_panel(), 0)
        main_layout.addWidget(self.create_plot_panel(), 1)

    def create_control_panel(self):
        panel = QWidget()
        panel.setFixedWidth(350)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

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
        self.control_stack.addWidget(self.create_control_scroll_area(self.create_overview_page()))
        self.control_stack.addWidget(self.create_control_scroll_area(self.create_debug_page()))
        layout.addWidget(self.control_stack, 1)
        layout.addWidget(self.create_terminal_group(), 0)
        return panel

    def create_control_scroll_area(self, page):
        page.setMinimumHeight(page.sizeHint().height())

        scroll_area = QScrollArea()
        scroll_area.setWidget(page)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QScrollArea.NoFrame)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        return scroll_area

    def make_nav_button(self, text):
        button = QPushButton(text)
        button.setCheckable(True)
        button.setCursor(Qt.PointingHandCursor)
        return button

    def create_overview_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        layout.addWidget(self.create_connection_group())
        layout.addWidget(self.create_motor_identity_group())
        layout.addWidget(self.create_realtime_group())
        layout.addWidget(self.create_quick_test_group())
        layout.addWidget(self.create_stats_group())
        layout.addStretch()
        return page

    def create_debug_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        layout.addWidget(self.create_motor_identity_group())
        layout.addWidget(self.create_sensor_group())
        layout.addWidget(self.create_test_group())
        layout.addWidget(self.create_command_group())
        layout.addStretch()
        return page

    def create_connection_group(self):
        group = QGroupBox("系统连接")
        layout = QGridLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)

        self.btn_connect = QPushButton("连接服务器")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.lbl_status = QLabel()
        self.lbl_status.setProperty("role", "value")
        self.lbl_endpoint = QLabel(f"{SERVER_HOST}:{SERVER_PORT}")
        self.lbl_endpoint.setProperty("role", "muted")

        layout.addWidget(self.btn_connect, 0, 0, 2, 1)
        layout.addWidget(self.lbl_status, 0, 1)
        layout.addWidget(self.lbl_endpoint, 1, 1)
        return group

    def create_realtime_group(self):
        group = QGroupBox("实时数据")
        grid = QGridLayout(group)
        grid.setContentsMargins(6, 6, 6, 6)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)

        headers = ("", "电机 1", "电机 2")
        for col, text in enumerate(headers):
            label = QLabel(text)
            label.setProperty("role", "muted")
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter if col else Qt.AlignLeft)
            grid.addWidget(label, 0, col)

        metrics = (
            ("torque", "扭矩 (Nm)", "0.00"),
            ("position", "位置 (deg)", "0.00"),
            ("velocity", "速度 (deg/s)", "0.00"),
            ("temp_mos", "MOS 温度 (℃)", "0.00"),
            ("temp_rotor", "线圈温度 (℃)", "0.00"),
        )
        for row, (metric, title, initial_value) in enumerate(metrics, start=1):
            metric_label = QLabel(title)
            metric_label.setProperty("role", "muted")
            grid.addWidget(metric_label, row, 0)

            for col, motor_key in enumerate(("motor1", "motor2"), start=1):
                value_label = QLabel(initial_value)
                value_label.setProperty("role", "value")
                value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                value_label.setMinimumWidth(108)
                value_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                self.value_labels[f"{motor_key}_{metric}"] = value_label
                grid.addWidget(value_label, row, col)

        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        return group

    def add_motor_row(self, grid, row, key, title):
        grid.addWidget(QLabel(title), row, 0)
        for col, metric in enumerate(("torque", "position", "velocity"), start=1):
            label = QLabel("0.00")
            label.setProperty("role", "value")
            self.value_labels[f"{key}_{metric}"] = label
            grid.addWidget(label, row, col)

    def create_quick_test_group(self):
        group = QGroupBox("测试")
        layout = QGridLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)

        self.btn_overview_test = QPushButton("一键测试")
        self.btn_overview_test.clicked.connect(self.run_test)
        self.test_buttons.append(self.btn_overview_test)
        self.progress_overview, self.lbl_overview_test_status = self.create_test_progress_widgets()

        layout.addWidget(self.btn_overview_test, 0, 0)
        layout.addWidget(self.progress_overview, 0, 1)
        layout.addWidget(self.lbl_overview_test_status, 1, 0, 1, 2)
        return group

    def create_test_progress_widgets(self):
        progress = QProgressBar()
        progress.setRange(0, 100)
        progress.setValue(0)
        progress.setFormat("%p%")
        status = QLabel("待机")
        status.setProperty("role", "muted")

        self.test_progress_bars.append(progress)
        self.test_status_labels.append(status)
        return progress, status

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
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)

        self.lbl_sensor_overview_status = QLabel("未连接")
        self.lbl_sensor_overview_status.setProperty("role", "value")
        self.lbl_sensor_torque = QLabel("-- Nm")
        self.lbl_sensor_torque.setProperty("role", "value")
        self.lbl_sensor_speed = QLabel("-- rpm")
        self.lbl_sensor_speed.setProperty("role", "value")
        self.lbl_sensor_peak_torque = QLabel("-- Nm")
        self.lbl_sensor_peak_torque.setProperty("role", "value")
        self.lbl_sensor_peak_speed = QLabel("-- rpm")
        self.lbl_sensor_peak_speed.setProperty("role", "value")
        self.sensor_status_labels.append(self.lbl_sensor_overview_status)

        layout.addWidget(QLabel("传感器状态"), 0, 0)
        layout.addWidget(self.lbl_sensor_overview_status, 0, 1)
        layout.addWidget(QLabel("传感器扭矩"), 1, 0)
        layout.addWidget(self.lbl_sensor_torque, 1, 1)
        layout.addWidget(QLabel("传感器转速"), 2, 0)
        layout.addWidget(self.lbl_sensor_speed, 2, 1)
        layout.addWidget(QLabel("扭矩最大值"), 3, 0)
        layout.addWidget(self.lbl_sensor_peak_torque, 3, 1)
        layout.addWidget(QLabel("转速最大值"), 4, 0)
        layout.addWidget(self.lbl_sensor_peak_speed, 4, 1)
        return group

    def create_motor_identity_group(self):
        group = QGroupBox("被测电机")
        layout = QGridLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)

        status_label = QLabel("未识别")
        status_label.setProperty("role", "value")
        model_label = QLabel("--")
        model_label.setProperty("role", "value")

        self.motor_detect_status_labels.append(status_label)
        self.motor_detect_model_labels.append(model_label)

        layout.addWidget(QLabel("识别状态"), 0, 0)
        layout.addWidget(status_label, 0, 1)
        layout.addWidget(QLabel("电机 1 型号"), 1, 0)
        layout.addWidget(model_label, 1, 1)
        self.update_motor_identity_ui()
        return group

    def create_sensor_group(self):
        group = QGroupBox("DYN200 传感器")
        layout = QGridLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)

        self.input_sensor_dev = QLineEdit(DEFAULT_DY200_DEVICE)
        self.input_sensor_baud = QLineEdit(str(DEFAULT_DY200_BAUD))
        self.btn_sensor_connect = QPushButton("连接传感器")
        self.btn_sensor_connect.clicked.connect(self.connect_dy200_sensor)
        self.lbl_sensor_status = QLabel("未连接")
        self.lbl_sensor_status.setProperty("role", "muted")
        self.sensor_status_labels.append(self.lbl_sensor_status)

        layout.addWidget(QLabel("串口"), 0, 0)
        layout.addWidget(self.input_sensor_dev, 0, 1)
        layout.addWidget(QLabel("波特率"), 1, 0)
        layout.addWidget(self.input_sensor_baud, 1, 1)
        layout.addWidget(self.btn_sensor_connect, 2, 0)
        layout.addWidget(self.lbl_sensor_status, 2, 1)
        return group

    def create_test_group(self):
        group = QGroupBox("对拖测试")
        layout = QGridLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)

        self.spin_test_pos = self.make_spinbox(-360.0, 360.0, DEFAULT_TEST_POS_DEG, 10.0, " deg")
        self.spin_test_vel = self.make_spinbox(1.0, 180.0, DEFAULT_TEST_VEL_DEG_S, 1.0, " deg/s")
        self.spin_drag_load = self.make_spinbox(0.0, 40.0, DEFAULT_LOAD_TORQUE_NM, 0.5, " Nm")
        self.lbl_test_estimate = QLabel()
        self.lbl_test_estimate.setProperty("role", "muted")
        self.progress_debug, self.lbl_debug_test_status = self.create_test_progress_widgets()

        for spin in (self.spin_test_pos, self.spin_test_vel, self.spin_drag_load):
            spin.valueChanged.connect(self.refresh_test_estimate)

        layout.addWidget(QLabel("相对角度"), 0, 0)
        layout.addWidget(self.spin_test_pos, 0, 1)
        layout.addWidget(QLabel("目标速度"), 1, 0)
        layout.addWidget(self.spin_test_vel, 1, 1)
        layout.addWidget(QLabel("负载扭矩"), 2, 0)
        layout.addWidget(self.spin_drag_load, 2, 1)
        layout.addWidget(self.lbl_test_estimate, 3, 0, 1, 2)

        self.btn_debug_test = QPushButton("启动往返测试")
        self.btn_debug_test.clicked.connect(self.run_test)
        self.test_buttons.append(self.btn_debug_test)
        layout.addWidget(self.btn_debug_test, 4, 0)
        layout.addWidget(self.progress_debug, 4, 1)
        layout.addWidget(self.lbl_debug_test_status, 5, 0, 1, 2)
        return group

    def create_command_group(self):
        group = QGroupBox("手动命令")
        layout = QGridLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(6)

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
        layout.addWidget(self.btn_forward, 0, 2)
        layout.addWidget(self.btn_reverse, 0, 3)
        return group

    def create_terminal_group(self):
        group = QGroupBox("后端终端")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        backend_layout = QHBoxLayout()
        backend_layout.setSpacing(6)
        self.btn_backend_start = QPushButton("启动后端")
        self.btn_backend_start.clicked.connect(self.start_backend_terminal)
        self.btn_backend_stop = QPushButton("停止后端")
        self.btn_backend_stop.clicked.connect(self.stop_backend_terminal)
        self.lbl_backend_status = QLabel("后端: 未启动")
        self.lbl_backend_status.setProperty("role", "muted")
        self.btn_terminal_expand = QPushButton()
        self.btn_terminal_expand.setIcon(self.make_expand_icon())
        self.btn_terminal_expand.setIconSize(QSize(20, 20))
        self.btn_terminal_expand.setFixedSize(34, 34)
        self.btn_terminal_expand.setToolTip("放大终端")
        self.btn_terminal_expand.clicked.connect(self.show_terminal_dialog)

        backend_layout.addWidget(self.btn_backend_start)
        backend_layout.addWidget(self.btn_backend_stop)
        backend_layout.addWidget(self.lbl_backend_status, 1)
        backend_layout.addWidget(self.btn_terminal_expand, 0, Qt.AlignRight)

        self.terminal_output = QPlainTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setFixedHeight(70)
        self.terminal_output.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        self.terminal_output.document().setMaximumBlockCount(TERMINAL_MAX_BLOCKS)

        input_layout = QHBoxLayout()
        input_layout.setSpacing(6)
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

    def make_expand_icon(self):
        pixmap = QPixmap(24, 24)
        pixmap.fill(Qt.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        pen = QPen(QColor("#eef1f5"), 2.4)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        painter.setPen(pen)

        painter.drawLine(QPoint(12, 12), QPoint(18, 6))
        painter.drawLine(QPoint(18, 6), QPoint(18, 12))
        painter.drawLine(QPoint(18, 6), QPoint(12, 6))

        painter.drawLine(QPoint(12, 12), QPoint(6, 18))
        painter.drawLine(QPoint(6, 18), QPoint(6, 12))
        painter.drawLine(QPoint(6, 18), QPoint(12, 18))
        painter.end()

        return QIcon(pixmap)

    def make_spinbox(self, minimum, maximum, value, step, suffix=""):
        spinbox = QDoubleSpinBox()
        spinbox.setRange(minimum, maximum)
        spinbox.setDecimals(2)
        spinbox.setSingleStep(step)
        spinbox.setValue(value)
        spinbox.setSuffix(suffix)
        spinbox.setMinimumHeight(36)
        return spinbox

    def create_plot_panel(self):
        panel = QWidget()
        layout = QGridLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        self.plot_torque = self.make_plot("扭矩反馈", "N·m")
        self.curve_torque1 = self.plot_torque.plot(pen=pg.mkPen("#f5c542", width=2), name="电机 1")
        self.curve_torque2 = self.plot_torque.plot(pen=pg.mkPen("#9b7bff", width=2), name="电机 2")
        self.curve_actual_torque = self.plot_torque.plot(
            pen=pg.mkPen("#4fc3f7", width=1, style=Qt.DashLine),
            name="模型",
        )
        self.curve_sensor_torque = self.plot_torque.plot(
            pen=pg.mkPen("#ff315a", width=4),
            name="DYN200 扭矩",
        )
        self.curve_sensor_torque.setZValue(20)
        self.line_sensor_peak_torque = pg.InfiniteLine(
            angle=0,
            movable=False,
            pen=pg.mkPen("#ff315a", width=2, style=Qt.DashLine),
        )
        self.line_sensor_peak_torque.setZValue(18)
        self.line_sensor_peak_torque.hide()
        self.plot_torque.addItem(self.line_sensor_peak_torque)

        self.plot_pos = self.make_plot("位置", "deg")
        self.curve_pos1 = self.plot_pos.plot(pen=pg.mkPen("#52d273", width=2), name="电机 1")
        self.curve_pos2 = self.plot_pos.plot(pen=pg.mkPen("#48c6d9", width=2), name="电机 2")
        self.curve_target_pos = self.plot_pos.plot(
            pen=pg.mkPen("#f58f42", width=1, style=Qt.DashLine),
            name="目标",
        )

        self.plot_vel = self.make_plot("速度 / 转速", "deg/s / rpm")
        self.curve_vel1 = self.plot_vel.plot(pen=pg.mkPen("#ff6b9a", width=2), name="电机 1")
        self.curve_vel2 = self.plot_vel.plot(pen=pg.mkPen("#75d377", width=2), name="电机 2")
        self.curve_target_vel = self.plot_vel.plot(
            pen=pg.mkPen("#57b8ff", width=1, style=Qt.DashLine),
            name="目标",
        )
        self.curve_sensor_speed = self.plot_vel.plot(
            pen=pg.mkPen("#ffffff", width=4),
            name="DYN200 转速",
        )
        self.curve_sensor_speed.setZValue(20)
        self.line_sensor_peak_speed = pg.InfiniteLine(
            angle=0,
            movable=False,
            pen=pg.mkPen("#ffffff", width=2, style=Qt.DashLine),
        )
        self.line_sensor_peak_speed.setZValue(18)
        self.line_sensor_peak_speed.hide()
        self.plot_vel.addItem(self.line_sensor_peak_speed)

        for plot in (self.plot_torque, self.plot_pos, self.plot_vel):
            plot.setMinimumHeight(170)

        layout.addWidget(self.plot_torque, 0, 0, 1, 2)
        layout.addWidget(self.plot_pos, 1, 0)
        layout.addWidget(self.plot_vel, 1, 1)
        layout.setRowStretch(0, 3)
        layout.setRowStretch(1, 2)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

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
            self.apply_pending_motor_preset()
            QTimer.singleShot(300, self.auto_connect_dy200_sensor)
        except OSError as exc:
            self.disconnect_all()
            self.append_terminal(f"! 连接失败: {exc}")
            self.lbl_status.setText(f"连接失败: {str(exc)[:28]}")
        finally:
            self.update_connection_ui()

    def disconnect_all(self):
        self.release_dy200_sensor()
        self.is_connected = False
        self.rx_buffer = b""
        self.test_token += 1
        self.test_in_progress = False
        self.applied_motor_model = None
        if self.detected_motor_model:
            self.pending_motor_model = self.detected_motor_model
        self.update_motor_identity_ui()
        self.progress_timer.stop()
        self.set_test_progress(0, "未连接")
        self.rx_timer.stop()

        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

        self.update_connection_ui()

    def release_dy200_sensor(self):
        if self.is_connected and self.sock:
            try:
                self.sock.sendall(b"DY200_DISCONNECT\n")
            except OSError:
                pass

        self.sensor_connected = False
        self.sensor_connecting = False
        self.sensor_auto_connect_attempted = False
        self.sensor_torque = np.nan
        self.sensor_speed = np.nan
        self.set_sensor_status("未连接")

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
        if self.terminal_dialog_input:
            self.terminal_dialog_input.setEnabled(running or self.is_connected)
        if self.terminal_dialog_send:
            self.terminal_dialog_send.setEnabled(running or self.is_connected)

    def show_terminal_dialog(self):
        if self.terminal_dialog and self.terminal_dialog.isVisible():
            self.terminal_dialog.raise_()
            self.terminal_dialog.activateWindow()
            return

        dialog = QDialog(self)
        dialog.setWindowTitle("后端终端")
        dialog.resize(1080, 720)
        dialog.setAttribute(Qt.WA_DeleteOnClose, True)

        layout = QVBoxLayout(dialog)
        output = QPlainTextEdit()
        output.setReadOnly(True)
        output.setLineWrapMode(QPlainTextEdit.WidgetWidth)
        output.document().setMaximumBlockCount(TERMINAL_MAX_BLOCKS * 4)
        output.setPlainText(self.terminal_output.toPlainText())
        cursor = output.textCursor()
        cursor.movePosition(QTextCursor.End)
        output.setTextCursor(cursor)

        input_layout = QHBoxLayout()
        terminal_input = QLineEdit()
        terminal_input.setPlaceholderText("输入后端命令后回车，例如: p_info")
        terminal_input.returnPressed.connect(self.send_terminal_dialog_command)
        send_button = QPushButton("回车")
        send_button.clicked.connect(self.send_terminal_dialog_command)
        clear_button = QPushButton("清空")
        clear_button.clicked.connect(self.clear_terminal)

        input_layout.addWidget(terminal_input, 1)
        input_layout.addWidget(send_button)
        input_layout.addWidget(clear_button)

        layout.addWidget(output)
        layout.addLayout(input_layout)

        self.terminal_dialog = dialog
        self.terminal_dialog_output = output
        self.terminal_dialog_input = terminal_input
        self.terminal_dialog_send = send_button
        dialog.finished.connect(self.on_terminal_dialog_closed)

        self.update_backend_ui()
        dialog.show()

    def on_terminal_dialog_closed(self, _result=None):
        self.terminal_dialog = None
        self.terminal_dialog_output = None
        self.terminal_dialog_input = None
        self.terminal_dialog_send = None

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
            self.parse_backend_terminal_text(text)
            if "TCP Server listening" in text and not self.is_connected:
                QTimer.singleShot(100, self.connect_server)

    def parse_backend_terminal_text(self, text):
        self.backend_parse_buffer += text
        lines = self.backend_parse_buffer.split("\n")
        self.backend_parse_buffer = lines.pop()

        if len(self.backend_parse_buffer) > 4096:
            self.backend_parse_buffer = self.backend_parse_buffer[-4096:]

        for line in lines:
            self.parse_backend_terminal_line(line.strip())

    def parse_backend_terminal_line(self, line):
        if not line:
            return

        can_match = MOTOR_CAN_ID_PATTERN.search(line)
        if can_match:
            source = can_match.group("source")
            can_id = int(can_match.group("can_id"))
            self.motor_source_can_ids[source] = can_id
            if can_id == 1 and source in self.motor_models_by_source:
                self.handle_detected_motor_model(self.motor_models_by_source[source])

        for program_match in MOTOR_PROGRAM_PATTERN.finditer(line):
            source = program_match.group("source")
            program = program_match.group("program")
            model = MOTOR_PROGRAM_TO_MODEL[program]
            self.motor_models_by_source[source] = model
            if self.motor_source_can_ids.get(source) == 1:
                self.handle_detected_motor_model(model)

    def update_connection_ui(self):
        connected = self.is_connected and self.sock is not None
        controls_enabled = connected and not self.test_in_progress
        self.lbl_status.setText("状态: 已连接" if connected else "状态: 未连接")
        self.lbl_status.setStyleSheet("color: #58d68d;" if connected else "color: #ff6b6b;")
        self.btn_connect.setText("断开连接" if connected else "连接服务器")
        self.btn_connect.setStyleSheet(
            "background-color: #9b2f35; border-color: #d96b72;"
            if connected
            else "background-color: #266f43; border-color: #58d68d;"
        )

        for button in (
            self.btn_zero,
            self.btn_disable,
            self.btn_forward,
            self.btn_reverse,
            *self.test_buttons,
        ):
            button.setEnabled(controls_enabled)
        if hasattr(self, "btn_sensor_connect"):
            self.btn_sensor_connect.setEnabled(
                connected and not self.test_in_progress and not self.sensor_connecting
            )
        self.update_backend_ui()

    def reset_can_log_join(self):
        self.last_can_log_id = None
        self.last_can_log_time = 0.0
        self.last_can_log_open = False

    def terminal_outputs(self):
        outputs = []
        if hasattr(self, "terminal_output"):
            outputs.append(self.terminal_output)
        if self.terminal_dialog_output:
            outputs.append(self.terminal_dialog_output)
        return outputs

    def scroll_terminal_widget_to_bottom(self, output):
        scroll_bar = output.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def scroll_terminal_to_bottom(self):
        for output in self.terminal_outputs():
            self.scroll_terminal_widget_to_bottom(output)

    def clear_terminal(self):
        for output in self.terminal_outputs():
            output.clear()
        self.reset_can_log_join()

    def append_terminal(self, message, reset_can_join=True):
        outputs = self.terminal_outputs()
        if not outputs:
            return
        for output in outputs:
            output.appendPlainText(message)
        self.scroll_terminal_to_bottom()
        if reset_can_join:
            self.reset_can_log_join()

    def append_terminal_raw(self, text):
        outputs = self.terminal_outputs()
        if not outputs:
            return
        for output in outputs:
            cursor = output.textCursor()
            cursor.movePosition(QTextCursor.End)
            cursor.insertText(text)
            output.setTextCursor(cursor)
        self.scroll_terminal_to_bottom()
        self.reset_can_log_join()

    def append_terminal_inline(self, text):
        outputs = self.terminal_outputs()
        if not outputs:
            return
        for output in outputs:
            cursor = output.textCursor()
            cursor.movePosition(QTextCursor.End)
            cursor.insertText(text)
            output.setTextCursor(cursor)
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
        self.submit_terminal_command(command)

    def send_terminal_dialog_command(self):
        if not self.terminal_dialog_input:
            return
        command = self.terminal_dialog_input.text()
        self.terminal_dialog_input.clear()
        self.submit_terminal_command(command)

    def submit_terminal_command(self, command):
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

    def auto_connect_dy200_sensor(self):
        if self.sensor_auto_connect_attempted or self.sensor_connected or self.sensor_connecting:
            return
        self.sensor_auto_connect_attempted = True
        self.connect_dy200_sensor(auto=True)

    def set_sensor_status(self, text):
        for label in self.sensor_status_labels:
            label.setText(text)

    def connect_dy200_sensor(self, auto=False):
        dev = self.input_sensor_dev.text().strip() or DEFAULT_DY200_DEVICE
        baud_text = self.input_sensor_baud.text().strip() or str(DEFAULT_DY200_BAUD)

        try:
            baud = int(baud_text)
        except ValueError:
            self.set_sensor_status("波特率无效")
            return

        if not self.is_connected:
            self.set_sensor_status("TCP 未连接")
            return

        if self.sensor_connecting:
            return

        if not os.path.exists(dev):
            self.set_sensor_status("未检测到串口" if auto else "串口不存在")
            return

        try:
            mode = os.stat(dev).st_mode
        except OSError as exc:
            self.set_sensor_status(f"串口检查失败: {exc.errno}")
            return

        if not stat.S_ISCHR(mode):
            self.set_sensor_status("不是串口设备")
            return

        if not os.access(dev, os.R_OK | os.W_OK):
            self.request_sensor_permission(dev, baud)
            return

        self.send_dy200_connect_command(dev, baud)

    def request_sensor_permission(self, dev, baud):
        if self.sensor_connecting:
            return

        self.sensor_connecting = True
        self.set_sensor_status("请求串口权限")
        self.update_connection_ui()

        process = QProcess(self)
        process.finished.connect(
            lambda exit_code, _status, d=dev, b=baud: self.on_sensor_permission_finished(
                exit_code, d, b
            )
        )
        process.errorOccurred.connect(self.on_sensor_permission_error)
        self.sensor_permission_process = process
        process.start("pkexec", ["chmod", "666", dev])

    def on_sensor_permission_finished(self, exit_code, dev, baud):
        self.sensor_permission_process = None
        self.sensor_connecting = False

        if exit_code != 0:
            self.set_sensor_status("赋权失败")
            self.update_connection_ui()
            return

        if not os.access(dev, os.R_OK | os.W_OK):
            self.set_sensor_status("赋权后仍无权限")
            self.update_connection_ui()
            return

        self.send_dy200_connect_command(dev, baud)

    def on_sensor_permission_error(self, _error):
        self.sensor_permission_process = None
        self.sensor_connecting = False
        self.set_sensor_status("赋权启动失败")
        self.update_connection_ui()

    def send_dy200_connect_command(self, dev, baud):
        if not self.is_connected:
            self.set_sensor_status("TCP 未连接")
            return

        self.sensor_torque = np.nan
        self.sensor_speed = np.nan
        self.sensor_connected = False
        self.sensor_connecting = True
        self.set_sensor_status("连接命令已发送")
        if not self.send_command(f"DY200_CONNECT {dev} {baud}\n"):
            self.sensor_connecting = False
            self.set_sensor_status("发送失败")
        self.update_connection_ui()

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
        sensor_updated = False
        valid_sensor_updated = False
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
            if key == "DY200_CONNECTED":
                self.sensor_connected = True
                self.sensor_connecting = False
                self.set_sensor_status("已连接，等待数据")
                self.update_connection_ui()
                continue
            if key == "DY200_CONNECT_FAILED":
                self.sensor_connected = False
                self.sensor_connecting = False
                self.set_sensor_status("连接失败")
                self.update_connection_ui()
                continue
            if key == "DY200_DISCONNECTED":
                self.sensor_connected = False
                self.sensor_connecting = False
                self.sensor_torque = np.nan
                self.sensor_speed = np.nan
                self.set_sensor_status("未连接")
                self.update_connection_ui()
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
                self.handle_backend_status_log(raw_line[4:])
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
            elif key == "TEMP_MOS1":
                self.motor_1.temp_mos = value
            elif key == "TEMP_ROTOR1":
                self.motor_1.temp_rotor = value
            elif key == "TORQUE2":
                self.motor_2.torque = value
            elif key == "POS2":
                self.motor_2.position = value
            elif key == "VEL2":
                self.motor_2.velocity = value
            elif key == "TEMP_MOS2":
                self.motor_2.temp_mos = value
            elif key == "TEMP_ROTOR2":
                self.motor_2.temp_rotor = value
            elif key == "SENSOR_TORQUE":
                self.sensor_torque = value
                sensor_updated = True
                valid_sensor_updated = valid_sensor_updated or np.isfinite(value)
            elif key == "SENSOR_SPEED":
                self.sensor_speed = value
                sensor_updated = True
                valid_sensor_updated = valid_sensor_updated or np.isfinite(value)

        if valid_sensor_updated:
            self.sensor_connected = True
            self.set_sensor_status("已接收数据")
            self.update_sensor_peak_values()
        elif sensor_updated and self.sensor_connected:
            self.set_sensor_status("已连接，等待有效数据")

        self.peak_torque = max(
            self.peak_torque,
            abs(self.motor_1.torque),
            abs(self.motor_2.torque),
        )

    def handle_backend_status_log(self, message):
        if message.startswith("DYN200 connected"):
            self.sensor_connected = True
            self.sensor_connecting = False
            self.set_sensor_status("已连接，等待数据")
            self.update_connection_ui()
        elif message.startswith("DYN200 connect failed"):
            self.sensor_connected = False
            self.sensor_connecting = False
            self.set_sensor_status("连接失败")
            self.update_connection_ui()
        elif message.startswith("DYN200 disconnected"):
            self.sensor_connected = False
            self.sensor_connecting = False
            self.set_sensor_status("未连接")
            self.update_connection_ui()

    def format_sensor_value(self, value, suffix):
        if np.isfinite(value):
            return f"{value:.2f} {suffix}"
        return f"-- {suffix}"

    def update_sensor_peak_values(self):
        if not self.test_in_progress:
            return

        if np.isfinite(self.sensor_torque):
            if (
                not np.isfinite(self.sensor_peak_torque)
                or abs(self.sensor_torque) > abs(self.sensor_peak_torque)
            ):
                self.sensor_peak_torque = self.sensor_torque

        if np.isfinite(self.sensor_speed):
            speed = abs(self.sensor_speed)
            if not np.isfinite(self.sensor_peak_speed) or speed > self.sensor_peak_speed:
                self.sensor_peak_speed = speed

        self.update_sensor_peak_ui()

    def update_sensor_peak_ui(self):
        if hasattr(self, "lbl_sensor_peak_torque"):
            peak_torque_display = (
                abs(self.sensor_peak_torque)
                if np.isfinite(self.sensor_peak_torque)
                else np.nan
            )
            self.lbl_sensor_peak_torque.setText(
                self.format_sensor_value(peak_torque_display, "Nm")
            )
        if hasattr(self, "lbl_sensor_peak_speed"):
            self.lbl_sensor_peak_speed.setText(
                self.format_sensor_value(self.sensor_peak_speed, "rpm")
            )

        if hasattr(self, "line_sensor_peak_torque"):
            if np.isfinite(self.sensor_peak_torque):
                self.line_sensor_peak_torque.setValue(self.sensor_peak_torque)
                self.line_sensor_peak_torque.show()
            else:
                self.line_sensor_peak_torque.hide()

        if hasattr(self, "line_sensor_peak_speed"):
            if np.isfinite(self.sensor_peak_speed):
                self.line_sensor_peak_speed.setValue(self.sensor_peak_speed)
                self.line_sensor_peak_speed.show()
            else:
                self.line_sensor_peak_speed.hide()

    def reset_sensor_peak_values(self):
        self.sensor_peak_torque = np.nan
        self.sensor_peak_speed = np.nan
        self.update_sensor_peak_ui()

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
            "vel1": abs(self.motor_1.velocity),
            "vel2": abs(self.motor_2.velocity),
            "target_vel": self.target_vel,
            "sensor_torque": self.sensor_torque,
            "sensor_speed": self.sensor_speed,
        }
        for key, value in values.items():
            self.history[key] = np.roll(self.history[key], -1)
            self.history[key][-1] = value

    def update_labels(self):
        self.value_labels["motor1_torque"].setText(f"{self.motor_1.torque:.2f}")
        self.value_labels["motor1_position"].setText(f"{self.motor_1.position:.2f}")
        self.value_labels["motor1_velocity"].setText(f"{abs(self.motor_1.velocity):.2f}")
        self.value_labels["motor1_temp_mos"].setText(f"{self.motor_1.temp_mos:.2f}")
        self.value_labels["motor1_temp_rotor"].setText(f"{self.motor_1.temp_rotor:.2f}")
        self.value_labels["motor2_torque"].setText(f"{self.motor_2.torque:.2f}")
        self.value_labels["motor2_position"].setText(f"{self.motor_2.position:.2f}")
        self.value_labels["motor2_velocity"].setText(f"{abs(self.motor_2.velocity):.2f}")
        self.value_labels["motor2_temp_mos"].setText(f"{self.motor_2.temp_mos:.2f}")
        self.value_labels["motor2_temp_rotor"].setText(f"{self.motor_2.temp_rotor:.2f}")
        self.lbl_sensor_torque.setText(self.format_sensor_value(self.sensor_torque, "Nm"))
        self.lbl_sensor_speed.setText(self.format_sensor_value(self.sensor_speed, "rpm"))
        self.update_sensor_peak_ui()

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
        self.curve_sensor_torque.setData(self.time_axis, self.history["sensor_torque"])
        self.curve_sensor_speed.setData(self.time_axis, self.history["sensor_speed"])

    def handle_detected_motor_model(self, model):
        if model not in MOTOR_PRESETS:
            return
        if model == self.detected_motor_model and (
            model == self.applied_motor_model or model == self.pending_motor_model
        ):
            return

        self.detected_motor_model = model
        self.apply_motor_preset(model)

    def apply_pending_motor_preset(self):
        if self.pending_motor_model:
            self.apply_motor_preset(self.pending_motor_model)
        elif self.detected_motor_model and self.applied_motor_model != self.detected_motor_model:
            self.apply_motor_preset(self.detected_motor_model)

    def apply_motor_preset(self, model):
        name, max_torque, mass, arm = MOTOR_PRESETS[model]
        self.load_mass_kg = mass
        self.arm_length_m = arm
        self.detected_motor_max_torque = max_torque
        self.update_motor_identity_ui()

        if not self.is_connected:
            self.pending_motor_model = model
            self.update_motor_identity_ui()
            return

        if self.applied_motor_model == model and self.pending_motor_model is None:
            return

        if not self.send_command(f"SET_MAX_TORQUE {max_torque}\n"):
            self.pending_motor_model = model
            return

        self.applied_motor_model = model
        self.pending_motor_model = None
        self.append_terminal(
            f"! 自动识别电机1型号: {model}（{name}），已应用 {max_torque:.0f} Nm"
        )
        self.update_motor_identity_ui()

    def update_motor_identity_ui(self):
        if not self.motor_detect_status_labels:
            return

        if not self.detected_motor_model:
            status = "未识别"
            model = "--"
            torque = "--"
            for label in self.motor_detect_status_labels:
                label.setText(status)
            for label in self.motor_detect_model_labels:
                label.setText(model)
            for label in self.motor_detect_torque_labels:
                label.setText(torque)
            return

        status = "已识别"
        if self.pending_motor_model:
            status = "已识别，待应用"
        elif self.applied_motor_model == self.detected_motor_model:
            status = "已识别，已应用"

        model = self.detected_motor_model
        torque = f"{self.detected_motor_max_torque:.0f} Nm"
        for label in self.motor_detect_status_labels:
            label.setText(status)
        for label in self.motor_detect_model_labels:
            label.setText(model)
        for label in self.motor_detect_torque_labels:
            label.setText(torque)

    def refresh_test_estimate(self):
        if not hasattr(self, "spin_test_pos"):
            return
        duration = self.single_move_duration_ms() / 1000.0
        self.lbl_test_estimate.setText(f"单程约 {duration:.1f} s，往返约 {duration * 2.0:.1f} s")

    def single_move_duration_ms(self):
        velocity = max(abs(self.spin_test_vel.value()), 0.001)
        distance = abs(self.spin_test_pos.value())
        return int(distance / velocity * 1000)

    def set_test_progress(self, value, text):
        value = max(0, min(100, int(value)))
        for progress in self.test_progress_bars:
            progress.setValue(value)
        for label in self.test_status_labels:
            label.setText(text)

    def start_test_progress_animation(self, start_value, end_value, duration_ms, text):
        self.progress_start_value = float(start_value)
        self.progress_end_value = float(end_value)
        self.progress_start_time = time.monotonic()
        self.progress_duration_ms = max(int(duration_ms), 1)
        self.progress_text = text
        self.set_test_progress(start_value, text)
        self.progress_timer.start(PROGRESS_INTERVAL_MS)

    def update_test_progress_animation(self):
        elapsed_ms = (time.monotonic() - self.progress_start_time) * 1000.0
        ratio = min(max(elapsed_ms / self.progress_duration_ms, 0.0), 1.0)
        value = self.progress_start_value + (
            self.progress_end_value - self.progress_start_value
        ) * ratio
        self.set_test_progress(value, self.progress_text)
        if ratio >= 1.0:
            self.progress_timer.stop()

    def finish_test(self, text):
        self.progress_timer.stop()
        self.test_in_progress = False
        self.set_test_progress(100, text)
        self.update_connection_ui()

    def abort_test(self, token, text, disable_motor=True):
        if token == self.test_token:
            self.test_token += 1
        self.progress_timer.stop()
        self.test_in_progress = False
        if disable_motor and self.is_connected:
            self.motor_disable()
        self.set_test_progress(100, text)
        self.append_terminal(f"! {text}")
        self.update_connection_ui()

    def run_test(self):
        if not self.is_connected or self.test_in_progress:
            return

        self.test_token += 1
        token = self.test_token
        move_ms = self.single_move_duration_ms()

        self.test_in_progress = True
        self.reset_peak_torque()
        self.reset_sensor_peak_values()
        self.update_connection_ui()
        self.set_test_progress(5, "测试开始：电机失能")

        if not self.motor_disable():
            self.abort_test(token, "测试启动失败：发送失能命令失败", disable_motor=False)
            return

        self.set_test_progress(10, "置零")
        if not self.set_zero_position():
            self.abort_test(token, "测试启动失败：发送置零命令失败")
            return

        QTimer.singleShot(
            TEST_ZERO_DELAY_MS,
            lambda: self.start_test_move(token, 1.0, 10, 45, "正向运动中"),
        )
        QTimer.singleShot(
            TEST_ZERO_DELAY_MS + move_ms,
            lambda: self.run_if_current(token, lambda: self.set_test_progress(45, "正向结束等待")),
        )
        QTimer.singleShot(
            TEST_ZERO_DELAY_MS + move_ms + TEST_SETTLE_MS,
            lambda: self.start_test_move(token, -1.0, 50, 85, "反向运动中"),
        )
        QTimer.singleShot(
            TEST_ZERO_DELAY_MS + move_ms * 2 + TEST_SETTLE_MS,
            lambda: self.run_if_current(token, lambda: self.set_test_progress(85, "反向结束等待")),
        )
        QTimer.singleShot(
            TEST_ZERO_DELAY_MS + move_ms * 2 + TEST_SETTLE_MS * 2,
            lambda: self.start_post_test_position_check(token),
        )

    def start_test_move(self, token, direction, progress_start, progress_end, progress_text):
        if token != self.test_token or not self.is_connected or not self.test_in_progress:
            return

        if direction > 0.0 and abs(self.motor_1.position) > ZERO_GUARD_DEG:
            self.abort_test(token, "电机位置未置零，测试中止")
            return

        if self.move_once(direction):
            self.start_test_progress_animation(
                progress_start,
                progress_end,
                self.single_move_duration_ms(),
                progress_text,
            )
        else:
            self.abort_test(token, "测试中止：发送运动命令失败")

    def run_if_current(self, token, callback):
        if token == self.test_token and self.is_connected and self.test_in_progress:
            callback()

    def start_post_test_position_check(self, token):
        if token != self.test_token or not self.is_connected or not self.test_in_progress:
            return

        self.progress_timer.stop()
        self.set_test_progress(90, "后置位置检测：电机失能")
        if not self.motor_disable():
            self.abort_test(token, "后置位置检测失败：发送失能命令失败", disable_motor=False)
            return

        QTimer.singleShot(
            POSITION_CHECK_SETTLE_MS,
            lambda: self.capture_post_check_position(token),
        )

    def capture_post_check_position(self, token):
        if token != self.test_token or not self.is_connected or not self.test_in_progress:
            return

        self.post_check_start_pos = self.motor_1.position
        self.set_test_progress(95, "后置位置检测：重新使能")
        if not self.motor_enable():
            self.abort_test(token, "后置位置检测失败：发送使能命令失败")
            return

        QTimer.singleShot(
            POSITION_CHECK_SETTLE_MS,
            lambda: self.finish_post_test_position_check(token),
        )

    def finish_post_test_position_check(self, token):
        if token != self.test_token or not self.is_connected or not self.test_in_progress:
            return

        start_pos = self.post_check_start_pos
        end_pos = self.motor_1.position
        delta = abs(end_pos - start_pos)

        if delta > POSITION_CHECK_TOLERANCE_DEG:
            message = (
                "测试结束后位置检测未通过。\n"
                f"电机1失能位置: {start_pos:.2f} deg\n"
                f"重新使能位置: {end_pos:.2f} deg\n"
                f"变化量: {delta:.2f} deg，允许: {POSITION_CHECK_TOLERANCE_DEG:.2f} deg"
            )
            self.append_terminal(f"! {message.replace(chr(10), ' | ')}")
            QMessageBox.warning(self, "位置检测未通过", message)
            self.abort_test(token, "位置检测未通过", disable_motor=True)
            return

        if not self.motor_disable():
            self.abort_test(token, "测试完成但最终失能命令发送失败", disable_motor=False)
            return

        self.finish_test("测试完成，位置检测通过")

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

    def motor_enable(self):
        self.target_vel = 0.0
        return self.send_command(b"ENABLE_MOTOR\n")

    def set_zero_position(self):
        if self.send_command(b"ZERO\n"):
            self.target_pos = 0.0
            return True
        return False

    def reset_peak_torque(self):
        self.peak_torque = 0.0
        if hasattr(self, "lbl_peak_torque"):
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
