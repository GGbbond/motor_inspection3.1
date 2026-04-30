import sys
import os
import socket
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QGroupBox, QPushButton, 
                            QDoubleSpinBox, QGridLayout, QStackedWidget, QFrame)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import pyqtgraph as pg

class MotorControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # --- 核心变量初始化 ---
        self.init_variables()
        
        # --- UI 初始化 ---
        self.init_ui()
        
        # --- 定时器设置 ---
        self.timer = QTimer() # 绘图定时器
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)
        
        self.rx_timer = QTimer() # 数据接收定时器
        self.rx_timer.timeout.connect(self.receive_data)

    def init_variables(self):
        self.sock = None
        self.is_connected = False
        self.target_torque = 0.0
        self.target_pos = 0.0
        self.target_vel = 0.0
        self.current_torque_val1 = 0.0
        self.current_pos_val1 = 0.0
        self.current_vel_val1 = 0.0
        self.current_torque_val2 = 0.0
        self.current_pos_val2 = 0.0
        self.current_vel_val2 = 0.0
        self.last_pos_val = 0.0
        self.max_torque = 0.0
        self.actual_torque_val = 0.0
        
        # 物理参数
        self.L = 0.35 
        self.m_dumbell = 10.0
        self.m_arm = 3.0
        self.g = 9.81
        
        # 绘图数据
        self.data_length = 260
        self.time_axis = np.linspace(-26.0, 0.0, self.data_length)
        self.torque_data1 = np.zeros(self.data_length)
        self.torque_data2 = np.zeros(self.data_length)
        self.pos_data1 = np.zeros(self.data_length)
        self.pos_data2 = np.zeros(self.data_length)
        self.target_pos_data = np.zeros(self.data_length)
        self.vel_data1 = np.zeros(self.data_length)
        self.vel_data2 = np.zeros(self.data_length)
        self.target_vel_data = np.zeros(self.data_length)
        self.actual_torque_data = np.zeros(self.data_length)
        
        self.current_max_torque_btn = None

    def init_ui(self):
        self.setWindowTitle('电机控制系统 v2.0')
        self.setGeometry(100, 100, 1300, 850)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")

        # 主中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # ==========================================
        # 左侧区域：导航按钮 + 局部切换面板
        # ==========================================
        left_container = QWidget()
        left_container.setFixedWidth(320)
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(0, 0, 0, 0)

        # 1. 顶部切换导航栏
        nav_layout = QHBoxLayout()
        self.btn_nav_basic = QPushButton("基础控制")
        self.btn_nav_debug = QPushButton("高级调试")
        
        for btn in [self.btn_nav_basic, self.btn_nav_debug]:
            btn.setCheckable(True)
            btn.setFixedHeight(40)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setStyleSheet("""
                QPushButton { background-color: #333; border: none; font-weight: bold; }
                QPushButton:checked { background-color: #4CAF50; color: white; }
                QPushButton:hover:!checked { background-color: #444; }
            """)
            nav_layout.addWidget(btn)
        
        self.btn_nav_basic.setChecked(True)
        self.btn_nav_basic.clicked.connect(lambda: self.switch_panel(0))
        self.btn_nav_debug.clicked.connect(lambda: self.switch_panel(1))
        left_layout.addLayout(nav_layout)

        # 2. 局部切换的堆栈窗口
        self.control_stack = QStackedWidget()
        
        # --- 页面 1: 基础控制面板 ---
        self.page_basic = QWidget()
        self.init_basic_page()
        self.control_stack.addWidget(self.page_basic)
        
        # --- 页面 2: 高级调试面板 ---
        self.page_debug = QWidget()
        self.init_debug_page()
        self.control_stack.addWidget(self.page_debug)

        left_layout.addWidget(self.control_stack)
        main_layout.addWidget(left_container)

        # ==========================================
        # 右侧区域：固定不动的图表
        # ==========================================
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        
        # 扭矩图
        self.plot_torque = pg.PlotWidget(title="扭矩反馈 (N·m)")
        self.curve_torque1 = self.plot_torque.plot(pen='y', name='电机1反馈扭矩')
        self.curve_torque2 = self.plot_torque.plot(pen='m', name='电机2反馈扭矩')
        self.curve_actual_torque = self.plot_torque.plot(pen='b', name='计算扭矩')
        
        # 位置图
        self.plot_pos = pg.PlotWidget(title="实时位置 (deg)")
        self.curve_pos1 = self.plot_pos.plot(pen='g', name='电机1位置')
        self.curve_pos2 = self.plot_pos.plot(pen='c', name='电机2位置')
        self.curve_target_pos = self.plot_pos.plot(pen='c', name='目标位置')
        
        # 速度图
        self.plot_vel = pg.PlotWidget(title="实时速度 (deg/s)")
        self.curve_vel1 = self.plot_vel.plot(pen=pg.mkPen('magenta', width=2), name='电机1速度')
        self.curve_vel2 = self.plot_vel.plot(pen=pg.mkPen('green', width=2), name='电机2速度')
        self.curve_target_vel = self.plot_vel.plot(pen=pg.mkPen('cyan', width=1, style=Qt.DashLine), name='目标速度')

        for p in [self.plot_torque, self.plot_pos, self.plot_vel]:
            p.showGrid(x=True, y=True)
            p.addLegend()
            p.setBackground("#121212")
            right_layout.addWidget(p)

        main_layout.addWidget(right_container, 3)

    def init_basic_page(self):
        """构建基础控制页面"""
        layout = QVBoxLayout(self.page_basic)
        # 连接控制组
        conn_group = QGroupBox("系统连接")
        conn_layout = QVBoxLayout()
        self.btn_connect = QPushButton("连接电机服务器")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.btn_connect.setStyleSheet("background-color: #4CAF50; padding: 10px; font-weight: bold;")
        self.lbl_status = QLabel("状态: 未连接")
        self.lbl_status.setStyleSheet("color: #ff4444;")
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.lbl_status)
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)

        # 数据面板
        data_group = QGroupBox("实时数据摘要")
        data_layout = QVBoxLayout()
        self.lbl_torque_val = QLabel("当前扭矩: 0.00 Nm")
        self.lbl_pos_val = QLabel("当前位置: 0.00 deg")
        self.lbl_vel_val = QLabel("当前速度: 0.00 deg/s")
        for lbl in [self.lbl_torque_val, self.lbl_pos_val, self.lbl_vel_val]:
            lbl.setStyleSheet("font-size: 15px; color: #00ff00; padding: 5px;")
            data_layout.addWidget(lbl)
        data_group.setLayout(data_layout)
        layout.addWidget(data_group)

        # 物理模型参数
        model_group = QGroupBox("模型参数设置")
        model_layout = QVBoxLayout()
        
        model_layout.addWidget(QLabel("哑铃重量 (kg):"))
        self.spin_weight = QDoubleSpinBox()
        self.spin_weight.setRange(0, 50); self.spin_weight.setValue(self.m_dumbell)
        self.spin_weight.valueChanged.connect(self.update_phys_params)
        model_layout.addWidget(self.spin_weight)

        model_layout.addWidget(QLabel("力臂长度 (m):"))
        self.spin_arm = QDoubleSpinBox()
        self.spin_arm.setRange(0.1, 1.5); self.spin_arm.setValue(self.L)
        self.spin_arm.valueChanged.connect(self.update_phys_params)
        model_layout.addWidget(self.spin_arm)
        
        model_group.setLayout(model_layout)
        layout.addWidget(model_group)
        layout.addStretch()

    def init_debug_page(self):
        """构建高级调试页面"""
        layout = QVBoxLayout(self.page_debug)

        #数据面板
        debug_data_group = QGroupBox("实时数据")
        debug_data_layout = QVBoxLayout()
        self.lbl_debug_torque_val1 = QLabel("电机 1 当前扭矩: 0.00 Nm")
        self.lbl_debug_pos_val1 = QLabel("电机 1 当前位置: 0.00 deg")
        self.lbl_debug_vel_val1 = QLabel("电机 1 当前速度: 0.00 deg/s")
        self.lbl_debug_torque_val2 = QLabel("电机 2 当前扭矩: 0.00 Nm")
        self.lbl_debug_pos_val2 = QLabel("电机 2 当前位置: 0.00 deg")
        self.lbl_debug_vel_val2 = QLabel("电机 2 当前速度: 0.00 deg/s")
        for lbl in [self.lbl_debug_torque_val1, self.lbl_debug_pos_val1, self.lbl_debug_vel_val1, 
                    self.lbl_debug_torque_val2, self.lbl_debug_pos_val2, self.lbl_debug_vel_val2]:
            lbl.setStyleSheet("font-size: 15px; color: #ffffff; padding: 5px;")
            debug_data_layout.addWidget(lbl)
        debug_data_group.setLayout(debug_data_layout)
        layout.addWidget(debug_data_group)
        
        # 选型设置
        motor_group = QGroupBox("电机型号/Max Torque")
        motor_layout = QGridLayout()
        mt_configs = [
            ("50标准 (40Nm)", 40.0), ("50加长 (40Nm)", 40.0),
            ("70标准 (80Nm)", 80.0), ("85标准 (160Nm)", 160.0)
        ]
        self.mt_btns = []
        for i, (name, val) in enumerate(mt_configs):
            btn = QPushButton(name)
            btn.clicked.connect(lambda chk, v=val, b=btn: self.set_max_torque(v, b))
            btn.setStyleSheet("background-color: #2196F3; padding: 5px; font-size: 11px;")
            motor_layout.addWidget(btn, i//2, i%2)
            self.mt_btns.append(btn)
        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)

        # 自动化测试
        test_group = QGroupBox("自动化控制")
        test_layout = QVBoxLayout()
        
        self.btn_zero = QPushButton("绝对位置置零")
        self.btn_zero.clicked.connect(self.set_zero_position)
        self.btn_zero.setStyleSheet("background-color: #FF9800; padding: 8px;")
        
        self.btn_test = QPushButton("启动一键测试流程")
        self.btn_test.clicked.connect(self.run_test)
        self.btn_test.setStyleSheet("background-color: #E91E63; padding: 12px; font-weight: bold;")
        
        self.btn_disable = QPushButton("紧急失能电机")
        self.btn_disable.clicked.connect(self.motor_disable)
        self.btn_disable.setStyleSheet("background-color: #b71c1c; padding: 8px;")

        test_layout.addWidget(self.btn_zero)
        test_layout.addWidget(self.btn_test)
        test_layout.addWidget(self.btn_disable)
        test_group.setLayout(test_layout)
        layout.addWidget(test_group)

        # 统计数据
        stat_group = QGroupBox("统计数据")
        stat_layout = QVBoxLayout()
        self.lbl_peak_torque = QLabel("峰值扭矩: 0.00 Nm")
        self.btn_clear_peak = QPushButton("清除峰值")
        self.btn_clear_peak.clicked.connect(self.reset_max_torque)
        stat_layout.addWidget(self.lbl_peak_torque)
        stat_layout.addWidget(self.btn_clear_peak)
        stat_group.setLayout(stat_layout)
        layout.addWidget(stat_group)

        layout.addStretch()

    # ==========================================
    # 逻辑处理函数
    # ==========================================
    def switch_panel(self, index):
        self.control_stack.setCurrentIndex(index)
        self.btn_nav_basic.setChecked(index == 0)
        self.btn_nav_debug.setChecked(index == 1)

    def toggle_connection(self):
        if not self.is_connected:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2)
                self.sock.connect(('127.0.0.1', 9999))
                self.is_connected = True
                self.btn_connect.setText("断开连接")
                self.btn_connect.setStyleSheet("background-color: #f44336; padding: 10px;")
                self.lbl_status.setText("状态: 已连接")
                self.lbl_status.setStyleSheet("color: #4CAF50;")
                self.rx_timer.start(20)
            except Exception as e:
                self.lbl_status.setText(f"错误: {str(e)[:20]}...")
        else:
            self.disconnect_all()

    def disconnect_all(self):
        self.is_connected = False
        if self.sock:
            self.sock.close()
            self.sock = None
        self.btn_connect.setText("连接电机服务器")
        self.btn_connect.setStyleSheet("background-color: #4CAF50; padding: 10px;")
        self.lbl_status.setText("状态: 未连接")
        self.lbl_status.setStyleSheet("color: #ff4444;")
        self.rx_timer.stop()

    def receive_data(self):
        if not self.is_connected: return
        try:
            self.sock.setblocking(False)
            data = self.sock.recv(1024).decode('utf-8')
            for line in data.split('\n'):
                if line.startswith("TORQUE1"):
                    self.current_torque_val1 = float(line.split()[1])
                    if abs(self.current_torque_val1) > self.max_torque:
                        self.max_torque = abs(self.current_torque_val1)
                elif line.startswith("POS1"):
                    self.current_pos_val1 = float(line.split()[1])
                elif line.startswith("VEL1"):
                    self.current_vel_val1 = float(line.split()[1])
                elif line.startswith("POS_WITH_VEL_COMPLETE"):
                    self.target_vel = 0.0
                elif line.startswith("TORQUE2"):
                    self.current_torque_val2 = float(line.split()[1])
                elif line.startswith("POS2"):
                    self.current_pos_val2 = float(line.split()[1])
                elif line.startswith("VEL2"):
                    self.current_vel_val2 = float(line.split()[1])
        except: pass

    def update_data(self):
        if not self.is_connected: return
        
        # 计算速度
        # dt = 0.1
        # self.current_vel_val = abs((self.current_pos_val1 - self.last_pos_val) / dt)
        # self.last_pos_val = self.current_pos_val1

        # 物理计算
        theta_rad = self.current_pos_val1 * np.pi / 180
        self.actual_torque_val = (self.m_dumbell * self.L + self.m_arm * 0.25) * self.g * np.sin(theta_rad)

        # 更新缓冲区
        self.torque_data1 = np.roll(self.torque_data1, -1); self.torque_data1[-1] = self.current_torque_val1
        self.torque_data2 = np.roll(self.torque_data2, -1); self.torque_data2[-1] = self.current_torque_val2
        self.pos_data1 = np.roll(self.pos_data1, -1); self.pos_data1[-1] = self.current_pos_val1
        self.pos_data2 = np.roll(self.pos_data2, -1); self.pos_data2[-1] = self.current_pos_val2
        self.target_pos_data = np.roll(self.target_pos_data, -1); self.target_pos_data[-1] = self.target_pos
        self.vel_data1 = np.roll(self.vel_data1, -1); self.vel_data1[-1] = self.current_vel_val1
        self.vel_data2 = np.roll(self.vel_data2, -1); self.vel_data2[-1] = self.current_vel_val2
        self.target_vel_data = np.roll(self.target_vel_data, -1); self.target_vel_data[-1] = self.target_vel
        self.actual_torque_data = np.roll(self.actual_torque_data, -1); self.actual_torque_data[-1] = self.actual_torque_val

        # 刷新 UI 标签
        self.lbl_torque_val.setText(f"当前扭矩: {self.current_torque_val1:.2f} Nm")
        self.lbl_pos_val.setText(f"当前位置: {self.current_pos_val1:.2f} deg")
        self.lbl_vel_val.setText(f"当前速度: {self.current_vel_val1:.2f} deg/s")
        self.lbl_debug_torque_val1.setText(f"电机 1 当前扭矩: {self.current_torque_val1:.2f} Nm")
        self.lbl_debug_pos_val1.setText(f"电机 1 当前位置: {self.current_pos_val1:.2f} deg")
        self.lbl_debug_vel_val1.setText(f"电机 1 当前速度: {self.current_vel_val1:.2f} deg/s")
        self.lbl_debug_torque_val2.setText(f"电机 2 当前扭矩: {self.current_torque_val2:.2f} Nm")
        self.lbl_debug_pos_val2.setText(f"电机 2 当前位置: {self.current_pos_val2:.2f} deg")
        self.lbl_debug_vel_val2.setText(f"电机 2 当前速度: {self.current_vel_val2:.2f} deg/s")
        self.lbl_peak_torque.setText(f"峰值扭矩: {self.max_torque:.2f} Nm")

        # 刷新图表
        self.curve_torque1.setData(self.time_axis, self.torque_data1)
        self.curve_torque2.setData(self.time_axis, self.torque_data2)
        self.curve_actual_torque.setData(self.time_axis, self.actual_torque_data)
        self.curve_pos1.setData(self.time_axis, self.pos_data1)
        self.curve_pos2.setData(self.time_axis, self.pos_data2)
        self.curve_target_pos.setData(self.time_axis, self.target_pos_data)
        self.curve_vel1.setData(self.time_axis, self.vel_data1)
        self.curve_vel2.setData(self.time_axis, self.vel_data2)
        self.curve_target_vel.setData(self.time_axis, self.target_vel_data)

    def set_max_torque(self, value, btn):
        if not self.is_connected: return
        if self.current_max_torque_btn:
            self.current_max_torque_btn.setStyleSheet("background-color: #2196F3; padding: 5px;")
        btn.setStyleSheet("background-color: #0D47A1; border: 2px solid gold; padding: 5px;")
        self.current_max_torque_btn = btn
        
        try:
            self.sock.sendall(f"SET_MAX_TORQUE {value}\n".encode())
            # 自动调整物理模型参数
            if "50" in btn.text(): self.m_dumbell, self.L = 5.0, 0.45
            elif "70" in btn.text(): self.m_dumbell, self.L = 10.0, 0.35
            elif "85" in btn.text(): self.m_dumbell, self.L = 25.0, 0.50
            self.spin_weight.setValue(self.m_dumbell)
            self.spin_arm.setValue(self.L)
        except: self.disconnect_all()

    def run_test(self):
        if not self.is_connected: return
        try:
            self.sock.sendall(b"ZERO\n")
            self.target_pos = 0.0
            QTimer.singleShot(500, lambda: self.send_cmd(360.0, 0.0, 36.0))
            QTimer.singleShot(13000, lambda: self.send_cmd(-360.0, 0.0, 36.0))
        except: pass

    def send_cmd(self, p, t, v):
        if self.sock:
            self.sock.sendall(f"POS_WITH_VEL {p} {t} {v}\n".encode())
            self.target_pos += p
            self.target_vel = v

    def motor_disable(self):
        if self.sock: self.sock.sendall(b"DISABLE_MOTOR\n")
    def set_zero_position(self):
        if self.sock: self.sock.sendall(b"ZERO\n"); self.target_pos = 0.0
    def reset_max_torque(self): self.max_torque = 0.0
    def update_phys_params(self):
        self.m_dumbell = self.spin_weight.value()
        self.L = self.spin_arm.value()

    def closeEvent(self, event):
        self.disconnect_all()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont('Segoe UI', 9))
    window = MotorControlApp()
    window.show()
    sys.exit(app.exec_())