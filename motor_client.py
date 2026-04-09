import sys
import os
import subprocess
import socket
import time
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QGroupBox, QPushButton, 
                            QDoubleSpinBox, QGridLayout)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import pyqtgraph as pg

# --- 新增：资源路径处理函数 ---
def resource_path(relative_path):
    """ 获取资源绝对路径，适配 PyInstaller 打包后的路径 """
    if hasattr(sys, '_MEIPASS'):
        # PyInstaller 打包后的临时目录
        base_path = sys._MEIPASS
    else:
        # 正常开发环境
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)

class MotorControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # --- 新增：服务器进程句柄 ---
        self.server_process = None
        self.sock = None
        # ---------------------------
        
        # 初始化变量
        self.serial_port = None
        self.is_connected = False
        self.target_torque = 0.0
        self.target_pos = 0.0  # 仅用于图表显示的目标位置累计值
        self.control_target_pos = 0.0  # 发送给电机的当前位置目标值（可按需保持不变）
        self.pos_vel_pos_input = 0.0  # 位置与速度控制输入的增量
        self.target_vel = 0.0
        self.current_torque_val = 0.0
        self.current_vel_val = 0.0
        self.current_pos_val = 0.0
        self.last_pos_val = 0.0
        self.max_torque = 0.0
        self.actual_torque_val = 0.0
        self.pos_offset = 0.0  # 位置偏移，用于归零
        self.data_length = 100
        self.torque_data = np.zeros(self.data_length)
        self.target_torque_data = np.zeros(self.data_length)
        self.pos_data = np.zeros(self.data_length)
        self.target_pos_data = np.zeros(self.data_length)
        self.vel_data = np.zeros(self.data_length)
        self.target_vel_data = np.zeros(self.data_length)
        self.actual_torque_data = np.zeros(self.data_length)
        
        # 物理参数
        self.L = 0.40  # 哑铃力臂长度 (m)
        self.m_dumbell = 10.0  # 哑铃质量 (kg)
        self.m_arm = 3.0  # 摆臂质量 (kg)
        self.g = 9.81  # 重力加速度 (m/s²)
        
        # 初始化UI
        self.init_ui()
        
        # 设置定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(100)  # 100ms更新一次
        
        # --- 新增：接收数据定时器 ---
        self.rx_timer = QTimer()
        self.rx_timer.timeout.connect(self.receive_data)
        # ---------------------------
        
    def init_ui(self):
        self.setWindowTitle('电机控制系统')
        self.setGeometry(100, 100, 1200, 800)
        
        # 主窗口部件
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # 左侧控制面板
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setSpacing(15)
        
        # 连接控制
        connection_group = QGroupBox("连接控制")
        connection_layout = QGridLayout()
        
        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.btn_connect.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 8px;")
        
        self.lbl_status = QLabel("状态: 未连接")
        self.lbl_status.setStyleSheet("color: red; font-size: 14px;")
        
        connection_layout.addWidget(self.btn_connect, 0, 0)
        connection_layout.addWidget(self.lbl_status, 1, 0)
        connection_group.setLayout(connection_layout)
        control_layout.addWidget(connection_group)
        
        # # 位置与速度控制
        # pos_vel_group = QGroupBox("位置与速度控制")
        # pos_vel_layout = QGridLayout()

        # self.spin_pos_vel_pos = QDoubleSpinBox()
        # self.spin_pos_vel_pos.setRange(-360.0, 360.0)
        # self.spin_pos_vel_pos.setSingleStep(1.0)
        # self.spin_pos_vel_pos.setValue(0.0)
        # self.spin_pos_vel_pos.setStyleSheet("color: white;")
        # self.spin_pos_vel_pos.valueChanged.connect(self.update_target_pos_from_pos_vel)

        # self.spin_pos_vel_velocity = QDoubleSpinBox() 
        # self.spin_pos_vel_velocity.setRange(0.0, 90.0)
        # self.spin_pos_vel_velocity.setSingleStep(0.1)
        # self.spin_pos_vel_velocity.setValue(1.0)
        # self.spin_pos_vel_velocity.setStyleSheet("color: white;")

        # self.btn_pos_vel = QPushButton("执行位置与速度控制")
        # self.btn_pos_vel.setStyleSheet("background-color: #9C27B0; color: white; font-size: 12px; padding: 5px;")
        # self.btn_pos_vel.clicked.connect(self.on_pos_vel_clicked)

        # pos_vel_layout.addWidget(QLabel("目标位置 (deg):"), 0, 0)
        # pos_vel_layout.addWidget(self.spin_pos_vel_pos, 0, 1)
        # pos_vel_layout.addWidget(QLabel("目标速度 (deg/s):"), 1, 0)
        # pos_vel_layout.addWidget(self.spin_pos_vel_velocity, 1, 1)
        # pos_vel_layout.addWidget(self.btn_pos_vel, 2, 0, 1, 2)

        # pos_vel_group.setLayout(pos_vel_layout)
        # control_layout.addWidget(pos_vel_group)
        
        # 实时数据显示
        data_group = QGroupBox("实时数据")
        data_layout = QVBoxLayout()
        
        self.lbl_torque = QLabel("电机反馈扭矩: 0.00 N·m")
        self.lbl_target_torque = QLabel("目标电机反馈扭矩: 0.00 N·m")
        self.lbl_pos = QLabel("当前位置: 0.00 deg")
        self.lbl_target_pos = QLabel("目标位置: 0.00 deg")
        self.lbl_vel = QLabel("当前速度: 0.00 deg/s")
        self.lbl_target_vel = QLabel("目标速度: 0.00 deg/s")
        self.lbl_max_torque = QLabel("电机反馈扭矩最大值: 0.00 N·m")
        self.lbl_actual_torque = QLabel("位置计算出的扭矩: 0.00 N·m")
        
        # 设置标签样式
        for lbl in [self.lbl_torque, self.lbl_target_torque, self.lbl_pos, 
                    self.lbl_target_pos, self.lbl_vel, self.lbl_max_torque, self.lbl_actual_torque]:
            lbl.setStyleSheet("font-size: 16px; color: white;")
        
        # 添加标签到布局
        data_layout.addWidget(self.lbl_torque)
        data_layout.addWidget(self.lbl_max_torque)
        data_layout.addWidget(self.lbl_actual_torque)
        # data_layout.addWidget(self.lbl_target_torque)
        data_layout.addWidget(self.lbl_pos)
        data_layout.addWidget(self.lbl_target_pos)
        data_layout.addWidget(self.lbl_vel)
        data_layout.addWidget(self.lbl_target_vel)
        

        # 添加清零按钮
        self.btn_reset_max_torque = QPushButton("电机反馈扭矩最大值清零")
        self.btn_reset_max_torque.setStyleSheet("background-color: #0029A5; color: white; font-size: 12px; padding: 5px;")
        self.btn_reset_max_torque.clicked.connect(self.reset_max_torque)
        data_layout.addWidget(self.btn_reset_max_torque)
        
        # # 添加当前位置归零按钮
        # self.btn_zero_pos = QPushButton("当前位置置零(需在电机失能状态下使用)")
        # self.btn_zero_pos.setStyleSheet("background-color: #FF9800; color: white; font-size: 12px; padding: 5px;")
        # self.btn_zero_pos.clicked.connect(self.set_zero_position)
        # data_layout.addWidget(self.btn_zero_pos)
        
        # 添加哑铃重量输入框
        dumbell_layout = QHBoxLayout()
        dumbell_layout.addWidget(QLabel("哑铃重量 (kg):"))
        self.spin_dumbell_weight = QDoubleSpinBox()
        self.spin_dumbell_weight.setRange(0.0, 50.0)
        self.spin_dumbell_weight.setSingleStep(0.1)
        self.spin_dumbell_weight.setValue(self.m_dumbell)  # 使用默认值
        self.spin_dumbell_weight.setStyleSheet("color: white;")
        self.spin_dumbell_weight.valueChanged.connect(self.update_dumbell_weight)
        dumbell_layout.addWidget(self.spin_dumbell_weight)
        data_layout.addLayout(dumbell_layout)
        
        data_group.setLayout(data_layout)
        control_layout.addWidget(data_group)

        # #电机失能按钮
        # self.btn_disable_motor = QPushButton("电机失能")
        # self.btn_disable_motor.setStyleSheet("background-color: #D90000; color: white; font-size: 12px; padding: 5px;")
        # self.btn_disable_motor.clicked.connect(self.motor_disable)
        # control_layout.addWidget(self.btn_disable_motor)

        #一键测试功能组
        test_group = QGroupBox("点击开始测试:测试前确保周围人员安全")
        test_layout = QGridLayout()

        self.btn_test = QPushButton("一键测试")
        self.btn_test.setStyleSheet("background-color: #E91E63; color: white; font-size: 12px; padding: 5px;")
        self.btn_test.clicked.connect(self.run_test)
        test_layout.addWidget(self.btn_test, 0, 0)
        test_group.setLayout(test_layout)
        control_layout.addWidget(test_group)


        
        # 添加弹性空间
        control_layout.addStretch()
        
        # 右侧图表区域
        chart_panel = QWidget()
        chart_layout = QVBoxLayout(chart_panel)
        
        # 扭矩图表
        self.torque_plot = pg.PlotWidget(title="电机反馈扭矩曲线")
        self.torque_plot.setBackground('k')
        self.torque_plot.showGrid(x=True, y=True)
        self.torque_plot.setLabel('left', '扭矩', units='N·m')
        self.torque_plot.addLegend()
        
        self.curve_torque = self.torque_plot.plot(pen='y', name='电机反馈扭矩')
        # self.curve_target_torque = self.torque_plot.plot(pen='r', name='目标电机反馈扭矩')
        
        # 位置图表
        self.pos_plot = pg.PlotWidget(title="位置曲线")
        self.pos_plot.setBackground('k')
        self.pos_plot.showGrid(x=True, y=True)
        self.pos_plot.setLabel('left', '位置', units='deg')
        self.pos_plot.addLegend()
        
        self.curve_pos = self.pos_plot.plot(pen='g', name='实时位置')
        self.curve_target_pos = self.pos_plot.plot(pen='c', name='目标位置')

        # 速度图表
        self.vel_plot = pg.PlotWidget(title="速度曲线")
        self.vel_plot.setBackground('k')
        self.vel_plot.showGrid(x=True, y=True)
        self.vel_plot.setLabel('left', '速度', units='deg/s')
        self.vel_plot.addLegend()

        self.curve_vel = self.vel_plot.plot(pen=pg.mkPen('magenta', width=3), name='实时速度')
        self.curve_target_vel = self.vel_plot.plot(pen=pg.mkPen('cyan', width=2, style=Qt.DashLine), name='目标速度')
        
        # 实际扭矩图表
        self.actual_torque_plot = pg.PlotWidget(title="位置计算出的扭矩曲线")
        self.actual_torque_plot.setBackground('k')
        self.actual_torque_plot.showGrid(x=True, y=True)
        self.actual_torque_plot.setLabel('left', '扭矩', units='N·m')
        self.actual_torque_plot.addLegend()
        
        self.curve_actual_torque = self.actual_torque_plot.plot(pen='b', name='位置计算出的扭矩')
        
        # 添加图表到布局
        chart_layout.addWidget(self.torque_plot)
        chart_layout.addWidget(self.pos_plot)
        chart_layout.addWidget(self.vel_plot)
        chart_layout.addWidget(self.actual_torque_plot)
        
        # 添加左右面板到主布局
        main_layout.addWidget(control_panel, 1)
        main_layout.addWidget(chart_panel, 3)
        
        # 设置整体背景色
        main_widget.setStyleSheet("background-color: #2b2b2b;")
        
        # 设置标签颜色
        for child in control_panel.findChildren(QLabel):
            child.setStyleSheet("color: white; font-size: 14px;")
            
        # 设置组框样式
        for group in control_panel.findChildren(QGroupBox):
            group.setStyleSheet("QGroupBox { color: white; font-weight: bold; font-size: 14px; border: 1px solid gray; border-radius: 5px; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px; }")
    
    # --- 修改：toggle_connection 方法 ---
    def toggle_connection(self):
        if not self.is_connected:
            # 尝试连接
            try:
                # --- 新增：尝试 TCP 连接到服务器 ---
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect(('127.0.0.1', 9999)) # 确保端口与 C 服务器一致
                # --------------------------------

                self.is_connected = True
                self.btn_connect.setText("断开连接")
                self.btn_connect.setStyleSheet("background-color: #f44336; color: white; font-size: 14px; padding: 8px;")
                self.lbl_status.setText("状态: 已连接")
                self.lbl_status.setStyleSheet("color: green; font-size: 14px;")
                
                # 启动接收数据的定时器
                self.rx_timer.start(20) # 20ms 刷新一次
                
            except Exception as e:
                self.lbl_status.setText(f"连接失败: {str(e)}")
                self.lbl_status.setStyleSheet("color: red; font-size: 14px;")
                
        else:
            # 断开连接
            try:
                if self.sock:
                    self.sock.close()
                    self.sock = None

                self.is_connected = False
                self.btn_connect.setText("连接")
                self.btn_connect.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 8px;")
                self.lbl_status.setText("状态: 未连接")
                self.lbl_status.setStyleSheet("color: red; font-size: 14px;")
                
                # 停止接收数据的定时器
                self.rx_timer.stop()
            except Exception as e:
                self.lbl_status.setText(f"断开连接失败: {str(e)}")
                self.lbl_status.setStyleSheet("color: red; font-size: 14px;")
    # -------------------------------------

    # --- 修改：set_target_torque 方法 ---
    def set_target_torque(self, value):
        self.target_torque = value
        if self.is_connected and self.sock:
            try:
                cmd = f"SET_TORQUE {value}\n"
                self.sock.sendall(cmd.encode('utf-8'))
            except Exception as e:
                print(f"发送失败: {e}")
                self.disconnect()
    # -----------------------------------

    def update_target_pos_from_pos_vel(self, value):
        # 仅更新位置控制输入增量，不立即修改当前目标位置
        self.pos_vel_pos_input = value

    # --- 新增：position_with_velocity 方法 ---
    def position_with_velocity(self, pos, torque, velocity):
        """调用C程序中的position_with_velocity函数"""
        if self.is_connected and self.sock:
            try:
                cmd = f"POS_WITH_VEL {pos} {torque} {velocity}\n"
                self.sock.sendall(cmd.encode('utf-8'))
                self.target_pos += pos  # 更新图表显示的目标位置
                self.target_vel = velocity  # 更新目标速度
                print(f"Sent position with velocity command: pos={pos}, torque={torque}, velocity={velocity}")
            except Exception as e:
                print(f"发送失败: {e}")
                self.disconnect()
    # -----------------------------------------

    # --- 新增：on_pos_vel_clicked 方法 ---
    def on_pos_vel_clicked(self):
        """处理位置与速度控制按钮点击事件"""
        # 使用当前输入增量进行控制，并在图表中叠加显示目标位置
        # pos_increment = self.pos_vel_pos_input
        # velocity = self.spin_pos_vel_velocity.value()

        # # 图表中目标位置累加显示（增量变化）
        # self.target_pos += pos_increment

        # # 控制命令使用增量，避免每次叠加叠加导致过度移动
        # self.target_vel = velocity
        # self.position_with_velocity(pos_increment, self.target_torque, velocity)

        # 只在实际完成时归零，由 receive_data 处理
        # self.target_vel = 0.0

        # 可选：清空增量输入值，使下一次输入明确
        # self.spin_pos_vel_pos.setValue(0.0)
    # ------------------------------------

    #电机失能方法
    def motor_disable(self):
        if self.is_connected and self.sock:
            try:
                cmd = "DISABLE_MOTOR\n"
                self.sock.sendall(cmd.encode('utf-8'))
                print("Sent motor disable command")
            except Exception as e:
                print(f"发送失败: {e}")
                self.disconnect()

    #一键测试方法
    def run_test(self):
        if not self.is_connected or not self.sock:
            return

        # 先失能电机并发送清零命令，后续动作用定时器顺序执行
        self.motor_disable()
        self.set_zero_position()

        
        # 0.2s 后开始位置清零动作完成后继续执行
        QTimer.singleShot(200, lambda: self.position_with_velocity(360.0, 0.0, 36.0))
        # 13s 后执行第二次位置命令
        QTimer.singleShot(13000, lambda: self.position_with_velocity(-360.0, 0.0, 36.0))
        # 再过13s 失能电机
        QTimer.singleShot(26000, self.motor_disable)    
    

    # --- 新增：receive_data 方法 ---
    def receive_data(self):
        if not self.is_connected or not self.sock:
            return
        try:
            self.sock.setblocking(False)
            try:
                while True:
                    data = self.sock.recv(1024).decode('utf-8')
                    if not data:
                        break
                    
                    lines = data.split('\n')
                    for line in lines:
                        if line.startswith("TORQUE"):
                            parts = line.split()
                            if len(parts) == 2:
                                val = float(parts[1])
                                self.current_torque_val = val
                                # 更新最大扭矩
                                if abs(val) > self.max_torque:
                                    self.max_torque = abs(val)
                        elif line.startswith("POS"):
                            parts = line.split()
                            if len(parts) == 2:
                                val = float(parts[1])
                                self.current_pos_val = val - self.pos_offset
                        elif line.startswith("POS_WITH_VEL_COMPLETE"):
                            print("Position with velocity command completed")
                            self.target_vel = 0.0  # 目标速度归零
            except BlockingIOError:
                pass
        except Exception as e:
            print(f"Connection lost: {e}")
            self.disconnect()
    # ------------------------------

    # --- 新增：disconnect 方法 ---
    def disconnect(self):
        self.is_connected = False
        if self.sock:
            self.sock.close()
            self.sock = None
        if self.server_process:
            self.server_process.terminate()
            self.server_process.wait()
            self.server_process = None
        
        self.btn_connect.setText("连接")
        self.btn_connect.setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 8px;")
        self.lbl_status.setText("状态: 未连接")
        self.lbl_status.setStyleSheet("color: red; font-size: 14px;")
        self.rx_timer.stop()
    # ---------------------------

    # --- 修改：update_data 方法 ---
    def update_data(self):
        if self.is_connected:
            # 更新数据数组
            self.torque_data = np.roll(self.torque_data, -1)
            self.torque_data[-1] = self.current_torque_val
            
            self.target_torque_data = np.roll(self.target_torque_data, -1)
            self.target_torque_data[-1] = self.target_torque
            
            self.pos_data = np.roll(self.pos_data, -1)
            self.pos_data[-1] = self.current_pos_val

            # 计算速度（deg/s）
            dt = 0.1  # update_data 每100ms调用一次
            self.current_vel_val = abs((self.current_pos_val - self.last_pos_val) / dt) #当前速度的绝对值
            self.last_pos_val = self.current_pos_val
            self.vel_data = np.roll(self.vel_data, -1)
            self.vel_data[-1] = self.current_vel_val

            self.target_vel_data = np.roll(self.target_vel_data, -1)
            self.target_vel_data[-1] = self.target_vel
            
            self.target_pos_data = np.roll(self.target_pos_data, -1)
            self.target_pos_data[-1] = self.target_pos
            
            self.actual_torque_data = np.roll(self.actual_torque_data, -1)
            # 计算实际扭矩：考虑哑铃和摆臂重量的重力扭矩
            theta_rad = self.current_pos_val * np.pi / 180  # 角度转弧度
            self.actual_torque_val = (self.m_dumbell * self.L + self.m_arm * 0.5 / 2) * self.g * np.sin(theta_rad)
            self.actual_torque_data[-1] = self.actual_torque_val
            
            # 更新图表
            self.curve_torque.setData(self.torque_data)
            # self.curve_target_torque.setData(self.target_torque_data)
            self.curve_pos.setData(self.pos_data)
            self.curve_target_pos.setData(self.target_pos_data)
            self.curve_vel.setData(self.vel_data)
            self.curve_target_vel.setData(self.target_vel_data)
            self.curve_actual_torque.setData(self.actual_torque_data)
            
            # 更新标签
            self.lbl_torque.setText(f"电机反馈扭矩: {self.current_torque_val:.2f} N·m")
            self.lbl_target_torque.setText(f"目标电机反馈扭矩: {self.target_torque:.2f} N·m")
            self.lbl_pos.setText(f"当前位置: {self.current_pos_val:.2f} deg")
            self.lbl_target_pos.setText(f"目标位置: {self.target_pos:.2f} deg")
            self.lbl_vel.setText(f"当前速度: {self.current_vel_val:.2f} deg/s")
            self.lbl_target_vel.setText(f"目标速度: {self.target_vel:.2f} deg/s")
            self.lbl_max_torque.setText(f"电机反馈扭矩最大值: {self.max_torque:.2f} N·m")
            self.lbl_actual_torque.setText(f"位置计算出的扭矩: {self.actual_torque_val:.2f} N·m")
    # ------------------------------

    def reset_max_torque(self):
        self.max_torque = 0.0
        self.lbl_max_torque.setText(f"电机反馈扭矩最大值: {self.max_torque:.2f} N·m")
        
    def set_zero_position(self):
        # 向C端发送ZERO指令，以调用mit_protocol的Zero实现电机绝对位置归零
        if self.is_connected and self.sock:
            try:
                self.sock.sendall(b"ZERO\n")
            except Exception as e:
                print(f"发送零位命令失败: {e}")
                self.disconnect()
        # self.pos_offset = self.current_pos_val + self.pos_offset  # 累加偏移
        # self.current_pos_val = 0.0
        self.target_pos = 0.0  # 目标位置也归零
        self.pos_data = np.zeros(self.data_length)
        self.target_pos_data = np.zeros(self.data_length)
        self.lbl_pos.setText(f"当前位置: {self.current_pos_val:.2f} deg")
        self.lbl_target_pos.setText(f"目标位置: {self.target_pos:.2f} deg")
        
    def update_dumbell_weight(self, value):
        """更新哑铃重量"""
        self.m_dumbell = value
        
    # --- 新增：closeEvent 方法 ---
    def closeEvent(self, event):
        self.disconnect()
        event.accept()
    # ---------------------------

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorControlApp()
    window.show()
    sys.exit(app.exec_())
