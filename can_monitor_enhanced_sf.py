import sys
import time
import struct
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QTextEdit, QLabel, QComboBox,
                             QSpinBox, QDoubleSpinBox, QGroupBox, QGridLayout, QMessageBox, QTableWidget,
                             QTableWidgetItem, QTabWidget, QSplitter, QScrollArea, QCheckBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QTextCursor, QColor
import pyqtgraph as pg
import zlgcan

class CANDataParser:
    """CAN数据解析器"""
    
    @staticmethod
    def parse_frame_1111201(data):
        """解析第1帧数据"""
        if len(data) < 8:
            return None
            
        displacement_cmd = struct.unpack('>H', data[0:2])[0]  # 位移指令 (大端序)
        displacement_feedback = struct.unpack('>H', data[2:4])[0]  # 位移反馈 (大端序)
        motor_speed = struct.unpack('>h', data[4:6])[0]  # 电机转速 (有符号，大端序)
        pressure5 = struct.unpack('>H', data[6:8])[0]  # 压力5 (大端序)
        
        return {
            'displacement_cmd': displacement_cmd * 0.01,  # mm
            'displacement_feedback': displacement_feedback * 0.01,  # mm
            'motor_speed': motor_speed,  # r/min
            'pressure5': pressure5 * 0.1  # Mpa
        }
    
    @staticmethod
    def parse_frame_1111202(data):
        """解析第2帧数据"""
        if len(data) < 8:
            return None
            
        dc_voltage = struct.unpack('>H', data[0:2])[0]  # DC母线电压 (大端序)
        motor_id_current = struct.unpack('>h', data[2:4])[0]  # 电机Id电流 (有符号，大端序)
        motor_iq_current = struct.unpack('>h', data[4:6])[0]  # 电机Iq电流 (有符号，大端序)
        igbt_temp = struct.unpack('>h', data[6:8])[0]  # IGBT温度 (有符号，大端序)
        
        return {
            'dc_voltage': dc_voltage,  # V
            'motor_id_current': motor_id_current * 0.1,  # A
            'motor_iq_current': motor_iq_current * 0.1,  # A
            'igbt_temp': igbt_temp  # °C
        }
    
    @staticmethod
    def parse_frame_1111203(data):
        """解析第3帧数据"""
        if len(data) < 6:
            return None
            
        motor_ia_current = struct.unpack('>h', data[0:2])[0]  # 电机Ia电流 (有符号，大端序)
        motor_ib_current = struct.unpack('>h', data[2:4])[0]  # 电机Ib电流 (有符号，大端序)
        pt100_a = struct.unpack('>b', data[4:5])[0]  # PT100-A (有符号，大端序)
        pt100_b = struct.unpack('>b', data[5:6])[0]  # PT100-B (有符号，大端序)
        pt100_c = struct.unpack('>b', data[6:7])[0]  # PT100-C (有符号，大端序)
        
        return {
            'motor_ia_current': motor_ia_current * 0.1,  # A
            'motor_ib_current': motor_ib_current * 0.1,  # A
            'pt100_a': pt100_a,  # °C
            'pt100_b': pt100_b,  # °C
            'pt100_c': pt100_c  # °C
        }
    
    @staticmethod
    def parse_frame_1111204(data):
        """解析第4帧数据"""
        if len(data) < 8:
            return None
            
        pressure1 = struct.unpack('>H', data[0:2])[0]  # 压力1 (大端序)
        pressure2 = struct.unpack('>H', data[2:4])[0]  # 压力2 (大端序)
        pressure3 = struct.unpack('>H', data[4:6])[0]  # 压力3 (大端序)
        pressure4 = struct.unpack('>H', data[6:8])[0]  # 压力4 (大端序)
        
        return {
            'pressure1': pressure1 * 0.1,  # Mpa
            'pressure2': pressure2 * 0.1,  # Mpa
            'pressure3': pressure3 * 0.1,  # Mpa
            'pressure4': pressure4 * 0.1  # Mpa
        }
    
    @staticmethod
    def parse_frame_1111205(data):
        """解析第5帧数据 (报警信息和电磁阀状态)"""
        if len(data) < 2:
            return None
            
        # Byte0: 报警信息
        alarm_byte = data[0]
        alarms = {
            'low_pressure_low': bool(alarm_byte & 0x01),      # Bit0: 低压过低报警
            'low_pressure_high': bool(alarm_byte & 0x02),     # Bit1: 低压过高报警
            'displacement_limit': bool(alarm_byte & 0x04),    # Bit2: 位移超限报警
            'motor_overspeed': bool(alarm_byte & 0x08),       # Bit3: 电机超速报警
            'motor_overcurrent': bool(alarm_byte & 0x10),     # Bit4: 电机过流报警
            'igbt_overtemp': bool(alarm_byte & 0x20),         # Bit5: IGBT过温报警
            'dc_voltage_over': bool(alarm_byte & 0x40),       # Bit6: 母线电压过压报警
            'dc_voltage_under': bool(alarm_byte & 0x80)       # Bit7: 母线电压欠压报警
        }
        
        # Byte1: 电磁阀状态和故障信息
        solenoid_byte = data[1]
        solenoid_status = {
            'solenoid0_status': bool(solenoid_byte & 0x01),   # Bit0: 电磁阀0状态 (1打开)
            'solenoid1_status': bool(solenoid_byte & 0x02),   # Bit1: 电磁阀1状态 (1打开)
            'solenoid2_status': bool(solenoid_byte & 0x04),   # Bit2: 电磁阀2状态 (1打开)
            'solenoid0_fault': bool(solenoid_byte & 0x08),    # Bit3: 电磁阀0故障报警
            'solenoid1_fault': bool(solenoid_byte & 0x10),    # Bit4: 电磁阀1故障报警
            'solenoid2_fault': bool(solenoid_byte & 0x20),    # Bit5: 电磁阀2故障报警
        }
        
        return {
            'alarms': alarms,
            'solenoid_status': solenoid_status
        }

class CANReceiverThread(QThread):
    """CAN数据接收线程"""
    data_received = pyqtSignal(int, bytes, int)  # can_id, data, timestamp
    raw_data_received = pyqtSignal(str)  # 原始数据字符串
    
    def __init__(self, zcanlib, chn_handle):
        super().__init__()
        self.zcanlib = zcanlib
        self.chn_handle = chn_handle
        self.running = False
        
    def run(self):
        self.running = True
        while self.running:
            try:
                rcv_num = self.zcanlib.GetReceiveNum(self.chn_handle, zlgcan.ZCAN_TYPE_CAN)
                if rcv_num > 0:
                    rcv_msg, rcv_num = self.zcanlib.Receive(self.chn_handle, rcv_num)
                    for i in range(rcv_num):
                        msg = rcv_msg[i]
                        data_bytes = bytes(msg.frame.data[:msg.frame.can_dlc])
                        
                        # 发送解析后的数据
                        self.data_received.emit(msg.frame.can_id, data_bytes, msg.timestamp)
                        
                        # 发送原始数据字符串
                        data_str = ' '.join([f'{b:02X}' for b in data_bytes])
                        timestamp = time.strftime('%H:%M:%S', time.localtime(msg.timestamp / 1000))
                        raw_str = f"[{timestamp}] ID: {msg.frame.can_id:03X} | DLC: {msg.frame.can_dlc} | Data: {data_str}"
                        self.raw_data_received.emit(raw_str)
                
                time.sleep(0.02)  # 20ms延时，进一步减少频率
            except Exception as e:
                self.raw_data_received.emit(f"接收错误: {str(e)}")
                break
    
    def stop(self):
        self.running = False


class PlotUpdateThread(QThread):
    """绘图更新线程"""
    plot_update_signal = pyqtSignal(str, object, object)  # param_name, time_array, value_array
    status_update_signal = pyqtSignal(str)  # 状态栏更新信号
    xrange_update_signal = pyqtSignal(float, float)  # start_time, end_time
    
    def __init__(self, plot_data, time_data, plot_curves, plot_widget):
        super().__init__()
        self.plot_data = plot_data
        self.time_data = time_data
        self.plot_curves = plot_curves
        self.plot_widget = plot_widget
        self.running = False
        self.pause_updates = False
        self.realtime_enabled = True
        self.current_tab = 0
        
    def set_pause_state(self, paused):
        """设置暂停状态"""
        self.pause_updates = paused
        
    def set_realtime_state(self, enabled):
        """设置实时更新状态"""
        self.realtime_enabled = enabled
        
    def set_current_tab(self, tab_index):
        """设置当前标签页"""
        self.current_tab = tab_index
        
    def run(self):
        """绘图更新线程主循环"""
        self.running = True
        update_counter = 0
        max_points = 1000
        while self.running:
            try:
                if self.pause_updates or not self.realtime_enabled:
                    time.sleep(0.2)  # 降低刷新频率到200ms
                    continue
                if len(self.time_data) > 0:
                    time_array = np.array(self.time_data, dtype=float)
                    updated_count = 0
                    for param_name in self.plot_curves:
                        if param_name in self.plot_data and len(self.plot_data[param_name]) > 0:
                            value_array = np.array(self.plot_data[param_name], dtype=float)
                            # 只取最近max_points
                            if len(time_array) > max_points:
                                time_array = time_array[-max_points:]
                                value_array = value_array[-max_points:]
                            if len(time_array) == len(value_array):
                                try:
                                    self.plot_update_signal.emit(param_name, time_array, value_array)
                                    updated_count += 1
                                except Exception as e:
                                    # print(f"更新曲线 {param_name} 时出错: {e}")
                                    continue
                    # if update_counter % 10 == 0:
                    #     print(f"绘图线程状态: 数据点={len(self.time_data)}, 更新曲线数={updated_count}, 暂停={self.pause_updates}, 实时={self.realtime_enabled}")
                    if updated_count > 0 and len(self.time_data) % 5 == 0:
                        try:
                            current_time = time_array[-1]
                            # 显示最近180秒（3分钟）的数据，确保能看到完整的时间轴
                            if current_time > 180:
                                start_time = current_time - 180
                                end_time = current_time + 10  # 增加10秒的前瞻
                            else:
                                start_time = 0
                                end_time = current_time + 10  # 增加10秒的前瞻
                            
                            # 确保end_time至少比当前时间大5秒，让曲线能够持续滚动
                            if end_time <= current_time + 5:
                                end_time = current_time + 10
                                
                            self.xrange_update_signal.emit(start_time, end_time)
                        except Exception as e:
                            # print(f"调整X轴范围时出错: {e}")
                            pass
                    if updated_count > 0 and len(self.time_data) % 20 == 0:
                        self.status_update_signal.emit(f"实时曲线更新中 - 数据点: {len(self.time_data)}")
                # else:
                #     if update_counter % 50 == 0:
                #         print(f"绘图线程等待数据: 暂停={self.pause_updates}, 实时={self.realtime_enabled}")
                update_counter += 1
                if update_counter % 200 == 0:
                    import gc
                    gc.collect()
                time.sleep(0.2)  # 降低刷新频率到200ms
            except Exception as e:
                # print(f"绘图更新线程出错: {e}")
                time.sleep(1)

    def stop(self):
        """停止线程"""
        self.running = False

class CANMonitorEnhancedGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.zcanlib = None
        self.device_handle = None
        self.chn_handle = None
        self.receiver_thread = None
        self.plot_update_thread = None
        self.parser = CANDataParser()
        
        # 初始化绘图更新暂停标志
        self.pause_plot_updates = False
        
        # 数据更新计数器，用于控制更新频率
        self.update_counter = 0
        
        # 初始化绘图数据存储（在UI创建之前）
        self.time_data = []
        self.plot_data = {}
        self.plot_curves = {}
        self.start_time = None
        self.plot_update_counter = 0
        
        # 初始化UI
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("50KW伺服控制器CAN数据监测")
        self.setGeometry(100, 100, 1400, 900)
        
        # 主窗口部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 设备控制区域
        self.create_device_control_group(main_layout)
        
        # 发送指令区域
        self.create_send_command_group(main_layout)
        
        # 创建标签页
        self.create_tab_widget(main_layout)
        
        # 控制按钮区域
        self.create_control_buttons(main_layout)
        
        # 状态栏
        self.statusBar().showMessage("就绪")
        
    def create_device_control_group(self, parent_layout):
        """创建设备控制组"""
        group = QGroupBox("设备控制")
        layout = QGridLayout()
        
        # 设备类型选择
        layout.addWidget(QLabel("设备类型:"), 0, 0)
        self.device_combo = QComboBox()
        self.device_combo.addItems([
            "USBCAN-2E-U",
            "USBCAN-E-U", 
            "USBCAN-4E-U",
            "USBCAN-8E-U"
        ])
        layout.addWidget(self.device_combo, 0, 1)
        
        # 设备索引
        layout.addWidget(QLabel("设备索引:"), 0, 2)
        self.device_index_spin = QSpinBox()
        self.device_index_spin.setRange(0, 10)
        layout.addWidget(self.device_index_spin, 0, 3)
        
        # 通道选择
        layout.addWidget(QLabel("通道:"), 1, 0)
        self.channel_combo = QComboBox()
        self.channel_combo.addItems(["通道0", "通道1"])
        layout.addWidget(self.channel_combo, 1, 1)
        
        # 波特率选择
        layout.addWidget(QLabel("波特率:"), 1, 2)
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems([
            "100000", "250000", "500000", "1000000"
        ])
        self.baudrate_combo.setCurrentText("500000")
        layout.addWidget(self.baudrate_combo, 1, 3)
        
        # 打开设备按钮
        self.open_device_btn = QPushButton("打开设备")
        self.open_device_btn.clicked.connect(self.open_device)
        layout.addWidget(self.open_device_btn, 2, 0, 1, 2)
        
        # 关闭设备按钮
        self.close_device_btn = QPushButton("关闭设备")
        self.close_device_btn.clicked.connect(self.close_device)
        self.close_device_btn.setEnabled(False)
        layout.addWidget(self.close_device_btn, 2, 2, 1, 2)
        
        group.setLayout(layout)
        parent_layout.addWidget(group)
        
    def create_send_command_group(self, parent_layout):
        # 位移和速度指令组
        cmd_group = QGroupBox("位移/速度指令")
        cmd_layout = QHBoxLayout()
        
        # 指令类型选择
        self.cmd_type_combo = QComboBox()
        self.cmd_type_combo.addItems(["位移指令", "速度指令"])
        self.cmd_type_combo.currentTextChanged.connect(self.on_cmd_type_changed)
        cmd_layout.addWidget(QLabel("指令类型:"))
        cmd_layout.addWidget(self.cmd_type_combo)
        
        # 指令数值输入
        self.cmd_value_input = QDoubleSpinBox()
        self.cmd_value_input.setRange(-32768, 65535)
        self.cmd_value_input.setValue(0)
        self.cmd_value_input.setDecimals(2)  # 支持2位小数
        cmd_layout.addWidget(QLabel("指令数值:"))
        cmd_layout.addWidget(self.cmd_value_input)
        
        # 单位显示
        self.unit_label = QLabel("mm")
        cmd_layout.addWidget(self.unit_label)
        
        # 指令使能控制
        self.cmd_enable_checkbox = QCheckBox("指令使能")
        self.cmd_enable_checkbox.setChecked(True)  # 默认使能
        cmd_layout.addWidget(self.cmd_enable_checkbox)
        
        # 发送按钮
        self.send_cmd_btn = QPushButton("发送")
        self.send_cmd_btn.clicked.connect(self.send_command_frame)
        cmd_layout.addWidget(self.send_cmd_btn)
        
        cmd_group.setLayout(cmd_layout)
        parent_layout.addWidget(cmd_group)
        
        # 电磁阀控制组
        solenoid_group = QGroupBox("电磁阀控制")
        solenoid_layout = QGridLayout()
        
        # 电磁阀0控制
        solenoid_layout.addWidget(QLabel("电磁阀0:"), 0, 0)
        self.solenoid0_combo = QComboBox()
        self.solenoid0_combo.addItems(["关闭", "输出"])
        solenoid_layout.addWidget(self.solenoid0_combo, 0, 1)
        
        # 电磁阀1控制
        solenoid_layout.addWidget(QLabel("电磁阀1:"), 0, 2)
        self.solenoid1_combo = QComboBox()
        self.solenoid1_combo.addItems(["关闭", "输出"])
        solenoid_layout.addWidget(self.solenoid1_combo, 0, 3)
        
        # 电磁阀2控制
        solenoid_layout.addWidget(QLabel("电磁阀2:"), 1, 0)
        self.solenoid2_combo = QComboBox()
        self.solenoid2_combo.addItems(["关闭", "打开"])
        solenoid_layout.addWidget(self.solenoid2_combo, 1, 1)
        
        # 发送电磁阀指令按钮
        self.send_solenoid_btn = QPushButton("发送电磁阀指令")
        self.send_solenoid_btn.clicked.connect(self.send_solenoid_frame)
        solenoid_layout.addWidget(self.send_solenoid_btn, 1, 2, 1, 2)
        
        solenoid_group.setLayout(solenoid_layout)
        parent_layout.addWidget(solenoid_group)
        
        # 初始化时设置默认状态
        self.on_cmd_type_changed("位移指令")
        
    def on_cmd_type_changed(self, cmd_type):
        """指令类型改变时的处理"""
        if cmd_type == "位移指令":
            self.unit_label.setText("mm")
            self.cmd_value_input.setRange(0, 655.35)  # 位移范围：0-655.35mm
            self.cmd_value_input.setSuffix(" mm")
            self.cmd_value_input.setDecimals(2)  # 支持2位小数
            # 更新使能控制标签
            self.cmd_enable_checkbox.setText("位移使能")
        else:  # 速度指令
            self.unit_label.setText("r/min")
            self.cmd_value_input.setRange(-32768, 32767)  # 速度范围：-32768到32767 r/min (有符号)
            self.cmd_value_input.setSuffix(" r/min")
            self.cmd_value_input.setDecimals(0)  # 整数
            # 更新使能控制标签
            self.cmd_enable_checkbox.setText("速度使能")
        
    def create_tab_widget(self, parent_layout):
        """创建标签页"""
        self.tab_widget = QTabWidget()
        
        # 数据解析标签页
        self.create_data_analysis_tab()
        
        # 实时曲线标签页
        self.create_realtime_plot_tab()
        
        parent_layout.addWidget(self.tab_widget)
        
    def create_data_analysis_tab(self):
        """创建数据解析标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 创建分割器
        splitter = QSplitter(Qt.Vertical)
        
        # 上半部分：数据标签显示区域
        self.create_data_labels_widget(splitter)
        
        # 中间部分：报警信息
        self.create_alarm_widget(splitter)
        
        layout.addWidget(splitter)
        self.tab_widget.addTab(tab, "数据解析")
        
    def create_realtime_plot_tab(self):
        """创建实时曲线标签页"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 创建分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧：曲线选择和控制
        self.create_plot_control_widget(splitter)
        
        # 右侧：绘图区域
        self.create_plot_widget(splitter)
        
        layout.addWidget(splitter)
        self.tab_widget.addTab(tab, "实时曲线")
        
    def create_data_labels_widget(self, parent):
        """创建数据标签显示控件"""
        data_widget = QWidget()
        layout = QVBoxLayout(data_widget)
        
        # 数据标题
        data_label = QLabel("解析数据")
        data_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(data_label)
        
        # 数据标签显示部分（极致紧凑，每项一行）
        data_content = QWidget()
        grid_layout = QGridLayout(data_content)
        self.data_labels = {}
        data_items = [
            ("displacement_cmd", "位移指令", "mm"),
            ("displacement_feedback", "位移反馈", "mm"),
            ("motor_speed", "电机转速", "r/min"),
            ("pressure5", "压力5", "Mpa"),
            ("dc_voltage", "DC母线电压", "V"),
            ("motor_id_current", "电机Id电流", "A"),
            ("motor_iq_current", "电机Iq电流", "A"),
            ("igbt_temp", "IGBT温度", "°C"),
            ("motor_ia_current", "电机Ia电流", "A"),
            ("motor_ib_current", "电机Ib电流", "A"),
            ("pt100_a", "PT100-A", "°C"),
            ("pt100_b", "PT100-B", "°C"),
            ("pt100_c", "PT100-C", "°C"),
            ("pressure1", "压力1", "Mpa"),
            ("pressure2", "压力2", "Mpa"),
            ("pressure3", "压力3", "Mpa"),
            ("pressure4", "压力4", "Mpa"),
        ]
        row, col, max_cols = 0, 0, 6
        for param_name, param_label, unit in data_items:
            label = QLabel(f"{param_label}: -- {unit}")
            label.setFont(QFont("Arial", 10))  # 增大字体从7到10
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            label.setContentsMargins(2, 1, 2, 1)  # 增加边距，使文字更清晰
            grid_layout.addWidget(label, row, col)
            self.data_labels[param_name] = {'label': label, 'unit': unit, 'param_label': param_label}
            col += 1
            if col >= max_cols:
                col = 0
                row += 1
        layout.addWidget(data_content)
        parent.addWidget(data_widget)
        
    def create_alarm_widget(self, parent):
        """创建报警信息和电磁阀状态控件"""
        alarm_widget = QWidget()
        layout = QVBoxLayout(alarm_widget)
        
        # 报警信息标题
        alarm_label = QLabel("报警信息")
        alarm_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(alarm_label)
        
        # 报警信息标签显示部分（无滚动区域）
        alarm_content = QWidget()
        alarm_layout = QGridLayout(alarm_content)
        self.alarm_labels = {}
        alarm_items = [
            "低压过低报警", "低压过高报警", "位移超限报警", "电机超速报警",
            "电机过流报警", "IGBT过温报警", "母线电压过压报警", "母线电压欠压报警"
        ]
        row, col, max_cols = 0, 0, 6  # 减少每行列数从8到6，适应更大的字体
        for i, item in enumerate(alarm_items):
            item_widget = QWidget()
            item_layout = QVBoxLayout(item_widget)
            item_layout.setContentsMargins(3, 3, 3, 3)  # 增加边距
            name_label = QLabel(item)
            name_label.setFont(QFont("Arial", 10, QFont.Bold))  # 增大字体从8到10
            name_label.setAlignment(Qt.AlignCenter)
            item_layout.addWidget(name_label)
            status_label = QLabel("正常")
            status_label.setFont(QFont("Arial", 11))  # 增大字体从9到11
            status_label.setAlignment(Qt.AlignCenter)
            status_label.setMinimumWidth(50)  # 增加最小宽度
            status_label.setStyleSheet("QLabel { background-color: #90EE90; border: 1px solid #ccc; padding: 3px; }")
            item_layout.addWidget(status_label)
            self.alarm_labels[i] = {'name': name_label, 'status': status_label}
            alarm_layout.addWidget(item_widget, row, col)
            col += 1
            if col >= max_cols:
                col = 0
                row += 1
        layout.addWidget(alarm_content)
        
        # 电磁阀状态标题
        solenoid_label = QLabel("电磁阀状态")
        solenoid_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(solenoid_label)
        
        # 电磁阀状态显示部分
        solenoid_content = QWidget()
        solenoid_layout = QGridLayout(solenoid_content)
        self.solenoid_status_labels = {}
        solenoid_items = [
            ("电磁阀0状态", "solenoid0_status"),
            ("电磁阀1状态", "solenoid1_status"), 
            ("电磁阀2状态", "solenoid2_status"),
            ("电磁阀0故障", "solenoid0_fault"),
            ("电磁阀1故障", "solenoid1_fault"),
            ("电磁阀2故障", "solenoid2_fault")
        ]
        row, col, max_cols = 0, 0, 6
        for i, (item_name, item_key) in enumerate(solenoid_items):
            item_widget = QWidget()
            item_layout = QVBoxLayout(item_widget)
            item_layout.setContentsMargins(3, 3, 3, 3)
            name_label = QLabel(item_name)
            name_label.setFont(QFont("Arial", 10, QFont.Bold))
            name_label.setAlignment(Qt.AlignCenter)
            item_layout.addWidget(name_label)
            status_label = QLabel("关闭")
            status_label.setFont(QFont("Arial", 11))
            status_label.setAlignment(Qt.AlignCenter)
            status_label.setMinimumWidth(50)
            status_label.setStyleSheet("QLabel { background-color: #90EE90; border: 1px solid #ccc; padding: 3px; }")
            item_layout.addWidget(status_label)
            self.solenoid_status_labels[item_key] = {'name': name_label, 'status': status_label}
            solenoid_layout.addWidget(item_widget, row, col)
            col += 1
            if col >= max_cols:
                col = 0
                row += 1
        layout.addWidget(solenoid_content)
        parent.addWidget(alarm_widget)
        
    def create_plot_control_widget(self, parent):
        """创建绘图控制控件"""
        control_widget = QWidget()
        control_widget.setMaximumWidth(300)
        layout = QVBoxLayout(control_widget)
        
        # 绘图控制标题
        control_label = QLabel("曲线控制")
        control_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(control_label)
        
        # 实时更新开关
        self.realtime_checkbox = QCheckBox("实时更新")
        self.realtime_checkbox.setChecked(True)
        self.realtime_checkbox.stateChanged.connect(self.on_realtime_changed)
        layout.addWidget(self.realtime_checkbox)
        
        # 暂停更新开关
        self.pause_checkbox = QCheckBox("暂停更新")
        self.pause_checkbox.setChecked(False)
        self.pause_checkbox.stateChanged.connect(self.on_pause_changed)
        layout.addWidget(self.pause_checkbox)
        
        # 曲线选择区域（用QScrollArea包裹）
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(2)
        
        # 曲线颜色映射（与init_plot_curves一致）
        curve_colors = {
            "displacement_cmd": "#ff0000",   # 红色
            "displacement_feedback": "gray", # 绿色
            "motor_speed": "#0000ff",         # 蓝色
            "pressure5": "#00ffff",           # 青色
            "dc_voltage": "#ff00ff",          # 洋红
            "motor_id_current": "#ffff00",    # 黄色
            "motor_iq_current": "#000000",    # 黑色
            "igbt_temp": "orange",            # 橙色
            "motor_ia_current": "purple",     # 紫色
            "motor_ib_current": "brown",      # 棕色
            "pt100_a": "pink",                # 粉色
            "pt100_b": "gray",                # 灰色
            "pt100_c": "olive",               # 橄榄色
            "pressure1": "navy",              # 海军蓝
            "pressure2": "teal",              # 青绿色
            "pressure3": "maroon",            # 栗色
            "pressure4": "lime",              # 酸橙色
        }
        
        # 定义可选的曲线
        self.curve_checkboxes = {}
        curve_options = [
            ("displacement_cmd", "位移指令", "mm"),
            ("displacement_feedback", "位移反馈", "mm"),
            ("motor_speed", "电机转速", "r/min"),
            ("pressure5", "压力5", "Mpa"),
            ("dc_voltage", "DC母线电压", "V"),
            ("motor_id_current", "电机Id电流", "A"),
            ("motor_iq_current", "电机Iq电流", "A"),
            ("igbt_temp", "IGBT温度", "°C"),
            ("motor_ia_current", "电机Ia电流", "A"),
            ("motor_ib_current", "电机Ib电流", "A"),
            ("pt100_a", "PT100-A", "°C"),
            ("pt100_b", "PT100-B", "°C"),
            ("pt100_c", "PT100-C", "°C"),
            ("pressure1", "压力1", "Mpa"),
            ("pressure2", "压力2", "Mpa"),
            ("pressure3", "压力3", "Mpa"),
            ("pressure4", "压力4", "Mpa"),
        ]
        
        for param_name, param_label, unit in curve_options:
            color = curve_colors.get(param_name, '#000000')
            color_label = QLabel()
            color_label.setFixedSize(18, 10)
            color_label.setStyleSheet(f"background-color: {color}; border-radius: 2px; margin-right: 4px;")
            checkbox = QCheckBox(f"{param_label} ({unit})")
            checkbox.setChecked(True)  # 默认选中
            checkbox.stateChanged.connect(lambda state, name=param_name: self.on_curve_selection_changed(name, state))
            self.curve_checkboxes[param_name] = checkbox
            # 横向布局：色块+复选框
            hbox = QHBoxLayout()
            hbox.setContentsMargins(0, 0, 0, 0)
            hbox.setSpacing(4)
            hbox.addWidget(color_label)
            hbox.addWidget(checkbox)
            hbox.addStretch()
            wrapper = QWidget()
            wrapper.setLayout(hbox)
            scroll_layout.addWidget(wrapper)
        scroll_layout.addStretch()
        scroll_content.setLayout(scroll_layout)
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        # 添加弹性空间
        layout.addStretch()
        parent.addWidget(control_widget)
        
    def create_plot_widget(self, parent):
        """创建绘图控件"""
        plot_widget = QWidget()
        layout = QVBoxLayout(plot_widget)
        
        # 创建pyqtgraph绘图窗口
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')  # 白色背景
        self.plot_widget.showGrid(x=True, y=True)  # 显示网格
        self.plot_widget.setLabel('left', '数值 (r/min, mm, V, A, °C, Mpa)')
        self.plot_widget.setLabel('bottom', '时间 (s)')
        self.plot_widget.setTitle('CAN数据实时曲线')
        
        # 初始化绘图数据
        self.init_plot_data()
        
        # 初始化绘图曲线
        self.init_plot_curves()
        
        layout.addWidget(self.plot_widget)
        parent.addWidget(plot_widget)
        
    def init_plot_data(self):
        """初始化绘图数据"""
        # 初始化所有参数的数据列表
        param_names = [
            'displacement_cmd', 'displacement_feedback', 'motor_speed', 'pressure5',
            'dc_voltage', 'motor_id_current', 'motor_iq_current', 'igbt_temp',
            'motor_ia_current', 'motor_ib_current', 'pt100_a', 'pt100_b', 'pt100_c',
            'pressure1', 'pressure2', 'pressure3', 'pressure4'
        ]
        
        for param_name in param_names:
            self.plot_data[param_name] = []
            
    def init_plot_curves(self):
        """初始化绘图曲线"""
        # 定义曲线颜色映射（与UI颜色标注一致）
        curve_colors = {
            "displacement_cmd": "r",
            "displacement_feedback": "g", 
            "motor_speed": "b",
            "pressure5": "c",
            "dc_voltage": "m",
            "motor_id_current": "y",
            "motor_iq_current": "k",
            "igbt_temp": "orange",
            "motor_ia_current": "purple",
            "motor_ib_current": "brown",
            "pt100_a": "pink",
            "pt100_b": "gray",
            "pt100_c": "olive",
            "pressure1": "navy",
            "pressure2": "teal",
            "pressure3": "maroon",
            "pressure4": "lime",
        }
        
        param_names = list(self.plot_data.keys())
        
        for param_name in param_names:
            color = curve_colors.get(param_name, 'k')  # 默认黑色
            curve = self.plot_widget.plot(pen=color, name=param_name)
            self.plot_curves[param_name] = curve
            
    def on_realtime_changed(self, state):
        """实时更新开关改变"""
        enabled = state == Qt.Checked
        if hasattr(self, 'plot_update_thread') and self.plot_update_thread:
            self.plot_update_thread.set_realtime_state(enabled)
            
    def on_pause_changed(self, state):
        """暂停更新开关改变"""
        paused = state == Qt.Checked
        if hasattr(self, 'plot_update_thread') and self.plot_update_thread:
            self.plot_update_thread.set_pause_state(paused)
            
    def on_curve_selection_changed(self, param_name, state):
        """曲线选择改变"""
        visible = state == Qt.Checked
        if param_name in self.plot_curves:
            if visible:
                self.plot_curves[param_name].show()
            else:
                self.plot_curves[param_name].hide()
                
    def start_plot_update_thread(self):
        """启动绘图更新线程"""
        # 确保绘图数据已初始化
        if not hasattr(self, 'plot_data') or not self.plot_data:
            self.init_plot_data()
            
        # 确保绘图曲线已初始化
        if not hasattr(self, 'plot_curves') or not self.plot_curves:
            self.init_plot_curves()
            
        if not hasattr(self, 'plot_update_thread') or not self.plot_update_thread:
            self.plot_update_thread = PlotUpdateThread(
                self.plot_data, self.time_data, self.plot_curves, self.plot_widget
            )
            self.plot_update_thread.plot_update_signal.connect(self.update_plot_curve)
            self.plot_update_thread.status_update_signal.connect(self.statusBar().showMessage)
            self.plot_update_thread.xrange_update_signal.connect(self.update_plot_xrange)
            self.plot_update_thread.start()
            
    def update_plot_curve(self, param_name, time_array, value_array):
        """更新绘图曲线"""
        max_points = 1000
        if len(time_array) > max_points:
            time_array = time_array[-max_points:]
            value_array = value_array[-max_points:]
        if param_name in self.plot_curves:
            try:
                valid_mask = ~np.isnan(value_array)
                if np.any(valid_mask):
                    valid_time = time_array[valid_mask]
                    valid_values = value_array[valid_mask]
                    self.plot_curves[param_name].setData(valid_time, valid_values)
                    # print(f"更新曲线 {param_name}: {len(valid_time)} 个数据点")
            except Exception as e:
                # print(f"更新曲线 {param_name} 时出错: {e}")
                pass
        # else:
        #     print(f"警告: 找不到曲线 {param_name}")
                
    def update_plot_xrange(self, start_time, end_time):
        """更新绘图X轴范围"""
        try:
            self.plot_widget.setXRange(start_time, end_time)
        except Exception as e:
            print(f"更新X轴范围时出错: {e}")

    def create_control_buttons(self, parent_layout):
        """创建控制按钮"""
        button_layout = QHBoxLayout()
        
        # 开始接收按钮
        self.start_btn = QPushButton("开始接收")
        self.start_btn.clicked.connect(self.start_receiving)
        self.start_btn.setEnabled(False)
        button_layout.addWidget(self.start_btn)
        
        # 停止接收按钮
        self.stop_btn = QPushButton("停止接收")
        self.stop_btn.clicked.connect(self.on_stop_button_clicked)
        self.stop_btn.setEnabled(False)
        button_layout.addWidget(self.stop_btn)
        
        button_layout.addStretch()
        
        # 保存数据按钮
        self.save_btn = QPushButton("保存数据")
        self.save_btn.clicked.connect(self.save_data)
        button_layout.addWidget(self.save_btn)
        
        # 清空数据按钮
        self.clear_btn = QPushButton("清空数据")
        self.clear_btn.clicked.connect(self.clear_all_data)
        button_layout.addWidget(self.clear_btn)
        
        parent_layout.addLayout(button_layout)

    def open_device(self):
        """打开CAN设备"""
        try:
            # 创建ZCAN实例
            self.zcanlib = zlgcan.ZCAN()
            
            # 获取设备类型
            device_type_map = {
                "USBCAN-2E-U": zlgcan.ZCAN_USBCAN_2E_U,
                "USBCAN-E-U": zlgcan.ZCAN_USBCAN_E_U,
                "USBCAN-4E-U": zlgcan.ZCAN_USBCAN_4E_U,
                "USBCAN-8E-U": zlgcan.ZCAN_USBCAN_8E_U
            }
            device_type = device_type_map[self.device_combo.currentText()]
            
            # 打开设备
            self.device_handle = self.zcanlib.OpenDevice(device_type, 
                                                        self.device_index_spin.value(), 0)
            
            if self.device_handle == zlgcan.INVALID_DEVICE_HANDLE:
                QMessageBox.critical(self, "错误", "打开设备失败!")
                return
                
            # 获取设备信息
            device_info = self.zcanlib.GetDeviceInf(self.device_handle)
            if device_info:
                self.statusBar().showMessage(f"设备已连接: {device_info.hw_type}")
            
            # 初始化通道
            channel_index = self.channel_combo.currentIndex()
            baudrate = self.baudrate_combo.currentText()
            
            # 设置波特率
            ret = self.zcanlib.ZCAN_SetValue(self.device_handle, 
                                           f"{channel_index}/baud_rate", 
                                           baudrate.encode("utf-8"))
            if ret != zlgcan.ZCAN_STATUS_OK:
                QMessageBox.warning(self, "警告", f"设置波特率失败: {ret}")
            
            # 初始化CAN通道
            chn_init_cfg = zlgcan.ZCAN_CHANNEL_INIT_CONFIG()
            chn_init_cfg.can_type = zlgcan.ZCAN_TYPE_CAN
            chn_init_cfg.config.can.acc_code = 0
            chn_init_cfg.config.can.acc_mask = 0xFFFFFFFF
            chn_init_cfg.config.can.mode = 0
            
            self.chn_handle = self.zcanlib.InitCAN(self.device_handle, channel_index, chn_init_cfg)
            if self.chn_handle == 0:
                QMessageBox.critical(self, "错误", "初始化CAN通道失败!")
                return
                
            # 启动CAN通道
            ret = self.zcanlib.StartCAN(self.chn_handle)
            if ret != zlgcan.ZCAN_STATUS_OK:
                QMessageBox.critical(self, "错误", "启动CAN通道失败!")
                return
                
            # 更新UI状态
            self.open_device_btn.setEnabled(False)
            self.close_device_btn.setEnabled(True)
            self.start_btn.setEnabled(True)
            self.device_combo.setEnabled(False)
            self.device_index_spin.setEnabled(False)
            self.channel_combo.setEnabled(False)
            self.baudrate_combo.setEnabled(False)
            
            QMessageBox.information(self, "成功", "设备连接成功!")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"打开设备时发生错误: {str(e)}")
            
    def close_device(self):
        """关闭CAN设备"""
        try:
            if self.receiver_thread and self.receiver_thread.isRunning():
                self.stop_receiving(update_ui=False)
                
            if self.chn_handle:
                self.zcanlib.ResetCAN(self.chn_handle)
                self.chn_handle = None
                
            if self.device_handle:
                self.zcanlib.CloseDevice(self.device_handle)
                self.device_handle = None
                
            # 更新UI状态
            self.open_device_btn.setEnabled(True)
            self.close_device_btn.setEnabled(False)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.device_combo.setEnabled(True)
            self.device_index_spin.setEnabled(True)
            self.channel_combo.setEnabled(True)
            self.baudrate_combo.setEnabled(True)
            
            self.statusBar().showMessage("设备已断开")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"关闭设备时发生错误: {str(e)}")
            
    def start_receiving(self):
        """开始接收数据"""
        if not self.chn_handle:
            QMessageBox.warning(self, "警告", "请先打开设备!")
            return
            
        try:
            # 重置时间基准和数据
            self.time_data.clear()
            for param_name in self.plot_data:
                self.plot_data[param_name].clear()
            self.start_time = None
            
            # 启动绘图更新线程
            self.start_plot_update_thread()
            
            self.receiver_thread = CANReceiverThread(self.zcanlib, self.chn_handle)
            self.receiver_thread.data_received.connect(self.on_data_received)
            self.receiver_thread.start()
            
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.statusBar().showMessage("正在接收数据...")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"启动接收线程失败: {str(e)}")
            
    def on_stop_button_clicked(self):
        """停止接收按钮点击处理"""
        self.stop_receiving(update_ui=True)
        
    def stop_receiving(self, update_ui=True):
        """停止接收数据"""
        if self.receiver_thread:
            self.receiver_thread.stop()
            self.receiver_thread.wait()
            self.receiver_thread = None
        # 停止绘图更新线程
        if hasattr(self, 'plot_update_thread') and self.plot_update_thread:
            self.plot_update_thread.stop()
            self.plot_update_thread.wait()
            self.plot_update_thread = None
        if update_ui:
            # 立即更新UI状态
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self.statusBar().showMessage("数据接收已停止")

        
    def on_data_received(self, can_id, data, timestamp):
        """处理接收到的解析数据"""
        # 只做数据解析和表格更新，不做帧计数
        if can_id == 0x1111201:
            self.update_frame_1111201(data)
        elif can_id == 0x1111202:
            self.update_frame_1111202(data)
        elif can_id == 0x1111203:
            self.update_frame_1111203(data)
        elif can_id == 0x1111204:
            self.update_frame_1111204(data)
        elif can_id == 0x1111205:
            self.update_frame_1111205(data)
            
    def update_frame_1111201(self, data):
        """更新第1帧数据"""
        parsed = self.parser.parse_frame_1111201(data)
        if parsed:
            if 'displacement_cmd' in self.data_labels:
                self.data_labels['displacement_cmd']['label'].setText(f"位移指令: {parsed['displacement_cmd']:.2f} mm")
            if 'displacement_feedback' in self.data_labels:
                self.data_labels['displacement_feedback']['label'].setText(f"位移反馈: {parsed['displacement_feedback']:.2f} mm")
            if 'motor_speed' in self.data_labels:
                self.data_labels['motor_speed']['label'].setText(f"电机转速: {parsed['motor_speed']} r/min")
            if 'pressure5' in self.data_labels:
                self.data_labels['pressure5']['label'].setText(f"压力5: {parsed['pressure5']:.1f} Mpa")
            self.append_data_point(parsed)
            
    def update_frame_1111202(self, data):
        """更新第2帧数据"""
        parsed = self.parser.parse_frame_1111202(data)
        if parsed:
            if 'dc_voltage' in self.data_labels:
                self.data_labels['dc_voltage']['label'].setText(f"DC母线电压: {parsed['dc_voltage']} V")
            if 'motor_id_current' in self.data_labels:
                self.data_labels['motor_id_current']['label'].setText(f"电机Id电流: {parsed['motor_id_current']:.1f} A")
            if 'motor_iq_current' in self.data_labels:
                self.data_labels['motor_iq_current']['label'].setText(f"电机Iq电流: {parsed['motor_iq_current']:.1f} A")
            if 'igbt_temp' in self.data_labels:
                self.data_labels['igbt_temp']['label'].setText(f"IGBT温度: {parsed['igbt_temp']} °C")
            self.append_data_point(parsed)
            
    def update_frame_1111203(self, data):
        """更新第3帧数据"""
        parsed = self.parser.parse_frame_1111203(data)
        if parsed:
            if 'motor_ia_current' in self.data_labels:
                self.data_labels['motor_ia_current']['label'].setText(f"电机Ia电流: {parsed['motor_ia_current']:.1f} A")
            if 'motor_ib_current' in self.data_labels:
                self.data_labels['motor_ib_current']['label'].setText(f"电机Ib电流: {parsed['motor_ib_current']:.1f} A")
            if 'pt100_a' in self.data_labels:
                self.data_labels['pt100_a']['label'].setText(f"PT100-A: {parsed['pt100_a']} °C")
            if 'pt100_b' in self.data_labels:
                self.data_labels['pt100_b']['label'].setText(f"PT100-B: {parsed['pt100_b']} °C")
            if 'pt100_c' in self.data_labels:
                self.data_labels['pt100_c']['label'].setText(f"PT100-C: {parsed['pt100_c']} °C")
            self.append_data_point(parsed)
            
    def update_frame_1111204(self, data):
        """更新第4帧数据"""
        parsed = self.parser.parse_frame_1111204(data)
        if parsed:
            if 'pressure1' in self.data_labels:
                self.data_labels['pressure1']['label'].setText(f"压力1: {parsed['pressure1']:.1f} Mpa")
            if 'pressure2' in self.data_labels:
                self.data_labels['pressure2']['label'].setText(f"压力2: {parsed['pressure2']:.1f} Mpa")
            if 'pressure3' in self.data_labels:
                self.data_labels['pressure3']['label'].setText(f"压力3: {parsed['pressure3']:.1f} Mpa")
            if 'pressure4' in self.data_labels:
                self.data_labels['pressure4']['label'].setText(f"压力4: {parsed['pressure4']:.1f} Mpa")
            self.append_data_point(parsed)
            
    def update_frame_1111205(self, data):
        """更新第5帧数据 (报警信息和电磁阀状态)"""
        parsed = self.parser.parse_frame_1111205(data)
        if parsed:
            # print("第5帧原始数据:", data)
            # print("第5帧解析结果:", parsed)
            # 更新报警信息
            alarms = parsed['alarms']
            alarm_items = [
                'low_pressure_low', 'low_pressure_high', 'displacement_limit', 'motor_overspeed',
                'motor_overcurrent', 'igbt_overtemp', 'dc_voltage_over', 'dc_voltage_under'
            ]
            for i, alarm_key in enumerate(alarm_items):
                if i in self.alarm_labels and alarm_key in alarms:
                    is_alarm = alarms[alarm_key]
                    if is_alarm:
                        self.alarm_labels[i]['status'].setText("报警")
                        self.alarm_labels[i]['status'].setStyleSheet("QLabel { background-color: #FFB6C1; border: 1px solid #ccc; padding: 3px; color: red; }")
                    else:
                        self.alarm_labels[i]['status'].setText("正常")
                        self.alarm_labels[i]['status'].setStyleSheet("QLabel { background-color: #90EE90; border: 1px solid #ccc; padding: 3px; color: green; }")
            
            # 更新电磁阀状态
            solenoid_status = parsed['solenoid_status']
            for status_key, status_info in self.solenoid_status_labels.items():
                if status_key in solenoid_status:
                    is_active = solenoid_status[status_key]
                    if 'fault' in status_key:
                        # 故障状态显示
                        if is_active:
                            status_info['status'].setText("故障")
                            status_info['status'].setStyleSheet("QLabel { background-color: #FFB6C1; border: 1px solid #ccc; padding: 3px; color: red; }")
                        else:
                            status_info['status'].setText("正常")
                            status_info['status'].setStyleSheet("QLabel { background-color: #90EE90; border: 1px solid #ccc; padding: 3px; color: green; }")
                    else:
                        # 电磁阀状态显示
                        if is_active:
                            status_info['status'].setText("打开")
                            status_info['status'].setStyleSheet("QLabel { background-color: #87CEEB; border: 1px solid #ccc; padding: 3px; color: blue; }")
                        else:
                            status_info['status'].setText("关闭")
                            status_info['status'].setStyleSheet("QLabel { background-color: #D3D3D3; border: 1px solid #ccc; padding: 3px; color: gray; }")
                    
    def clear_all_data(self):
        """清空所有数据"""
        for param_name in self.data_labels:
            label = self.data_labels[param_name]['label']
            param_label = self.data_labels[param_name]['param_label']
            unit = self.data_labels[param_name]['unit']
            label.setText(f"{param_label}: -- {unit}")
        for i in self.alarm_labels:
            self.alarm_labels[i]['status'].setText("正常")
            self.alarm_labels[i]['status'].setStyleSheet("QLabel { background-color: #90EE90; border: 1px solid #ccc; padding: 3px; color: green; }")
        # 清空电磁阀状态
        if hasattr(self, 'solenoid_status_labels'):
            for status_key, status_info in self.solenoid_status_labels.items():
                if 'fault' in status_key:
                    status_info['status'].setText("正常")
                    status_info['status'].setStyleSheet("QLabel { background-color: #90EE90; border: 1px solid #ccc; padding: 3px; color: green; }")
                else:
                    status_info['status'].setText("关闭")
                    status_info['status'].setStyleSheet("QLabel { background-color: #D3D3D3; border: 1px solid #ccc; padding: 3px; color: gray; }")
        
    def save_data(self):
        """保存数据到文件"""
        try:
            from PyQt5.QtWidgets import QFileDialog
            filename, _ = QFileDialog.getSaveFileName(
                self, "保存数据", "", "文本文件 (*.txt);;所有文件 (*)"
            )
            
            if filename:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write("=== CAN数据监控记录 ===\n")
                    f.write(f"记录时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                    
                    # 写入解析数据
                    f.write("=== 解析数据 ===\n")
                    # 定义数据项名称映射
                    param_names = {
                        'displacement_cmd': '位移指令',
                        'displacement_feedback': '位移反馈',
                        'motor_speed': '电机转速',
                        'pressure5': '压力5',
                        'dc_voltage': 'DC母线电压',
                        'motor_id_current': '电机Id电流',
                        'motor_iq_current': '电机Iq电流',
                        'igbt_temp': 'IGBT温度',
                        'motor_ia_current': '电机Ia电流',
                        'motor_ib_current': '电机Ib电流',
                        'pt100_a': 'PT100-A',
                        'pt100_b': 'PT100-B',
                        'pt100_c': 'PT100-C',
                        'pressure1': '压力1',
                        'pressure2': '压力2',
                        'pressure3': '压力3',
                        'pressure4': '压力4'
                    }
                    
                    for param_name, label_info in self.data_labels.items():
                        param_label = param_names.get(param_name, param_name)
                        value = label_info['label'].text().split(': ')[1]
                        f.write(f"{param_label}: {value}\n")
                    
                QMessageBox.information(self, "成功", f"数据已保存到: {filename}")
                
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存数据失败: {str(e)}")
            
    def send_command_frame(self):
        if not self.chn_handle:
            QMessageBox.warning(self, "警告", "请先打开设备!")
            return
            
        cmd_type = self.cmd_type_combo.currentText()
        value = self.cmd_value_input.value()  # QDoubleSpinBox返回float
        
        if cmd_type == "位移指令":
            can_id = 0x01121101
            data = bytearray(8)
            
            # 根据使能状态设置Byte0
            if self.cmd_enable_checkbox.isChecked():
                data[0] = 0xFF  # 指令有效，位移使能
                enable_status = "使能"
            else:
                data[0] = 0x00  # 指令无效，位移非使能
                enable_status = "非使能"
            
            # 位移指令：输入mm值，转换为0.01mm单位
            # 例如：输入10.5mm，发送1050 (10.5 * 100)
            raw_value = int(value * 100)  # 转换为0.01mm单位
            data[1:3] = raw_value.to_bytes(2, byteorder='big', signed=False)  # 大端序
            
            # 显示发送信息
            info_msg = f"发送位移指令: {value} mm (原始值: {raw_value}, 状态: {enable_status})"
            
        else:  # 速度指令
            can_id = 0x01121010
            data = bytearray(3)  # 只使用3个字节
            
            # 根据使能状态设置Byte0
            if self.cmd_enable_checkbox.isChecked():
                data[0] = 0xFF  # 指令有效，速度使能
                enable_status = "使能"
            else:
                data[0] = 0x00  # 指令无效，速度非使能
                enable_status = "非使能"
            
            # 速度指令：输入r/min值，有符号整型，大端序
            # 例如：输入1500 r/min，发送1500
            raw_value = int(value)
            data[1:3] = raw_value.to_bytes(2, byteorder='big', signed=True)  # 大端序，有符号
            
            # 显示发送信息
            info_msg = f"发送速度指令: {value} r/min (原始值: {raw_value}, 状态: {enable_status})"
        
        # 组帧
        frame = zlgcan.ZCAN_CAN_FRAME()
        frame.can_id = can_id
        frame.eff = 1 if can_id > 0x7FF else 0  # 扩展帧
        frame.rtr = 0  # 数据帧
        frame.can_dlc = len(data)  # 使用实际数据长度
        
        for i in range(len(data)):
            frame.data[i] = data[i]
            
        msg = zlgcan.ZCAN_Transmit_Data()
        msg.frame = frame
        msg.transmit_type = 0  # 正常发送
        
        
        # 打印实际发送的指令
        data_str = ' '.join([f'{b:02X}' for b in data])
        print(f"发送指令 - ID: {can_id:03X}, DLC: {len(data)}, Data: {data_str}")
        
        # 发送
        ret = self.zcanlib.Transmit(self.chn_handle, msg, 1)
        if ret == 1:
            self.statusBar().showMessage(f"{info_msg} - 发送成功", 3000)  # 显示3秒
        else:
            QMessageBox.warning(self, "失败", f"{info_msg}\n指令发送失败，返回值: {ret}")
            
    def send_solenoid_frame(self):
        """发送电磁阀控制指令"""
        if not self.chn_handle:
            QMessageBox.warning(self, "警告", "请先打开设备!")
            return
            
        # 获取电磁阀状态
        solenoid0_state = self.solenoid0_combo.currentText()
        solenoid1_state = self.solenoid1_combo.currentText()
        solenoid2_state = self.solenoid2_combo.currentText()
        
        # 构建数据帧
        can_id = 0x01121011
        data = bytearray(8)
        
        # Byte0: 指令有效
        data[0] = 0xFF
        
        # Byte1: 电磁阀0控制
        # 0xFF: 关闭, 0x00: 输出
        if solenoid0_state == "关闭":
            data[1] = 0xFF
        else:  # "输出"
            data[1] = 0x00
            
        # Byte2: 电磁阀1控制
        # 0xFF: 关闭, 0x00: 输出
        if solenoid1_state == "关闭":
            data[2] = 0xFF
        else:  # "输出"
            data[2] = 0x00
            
        # Byte3: 电磁阀2控制
        # 0xFF: 打开, 0x00: 关闭 (注意与电磁阀0和1是反的)
        if solenoid2_state == "打开":
            data[3] = 0xFF
        else:  # "关闭"
            data[3] = 0x00
            
        # Byte4-7: 保留字节，设为0
        data[4:8] = [0x00, 0x00, 0x00, 0x00]
        
        # 组帧
        frame = zlgcan.ZCAN_CAN_FRAME()
        frame.can_id = can_id
        frame.eff = 1 if can_id > 0x7FF else 0  # 扩展帧
        frame.rtr = 0  # 数据帧
        frame.can_dlc = 8
        
        for i in range(8):
            frame.data[i] = data[i]
            
        msg = zlgcan.ZCAN_Transmit_Data()
        msg.frame = frame
        msg.transmit_type = 0  # 正常发送
        
        # 打印实际发送的指令
        data_str = ' '.join([f'{b:02X}' for b in data])
        print(f"发送电磁阀指令 - ID: {can_id:03X}, DLC: {len(data)}, Data: {data_str}")
        
        # 发送
        ret = self.zcanlib.Transmit(self.chn_handle, msg, 1)
        if ret == 1:
            info_msg = f"电磁阀指令发送成功 - 电磁阀0:{solenoid0_state}, 电磁阀1:{solenoid1_state}, 电磁阀2:{solenoid2_state}"
            self.statusBar().showMessage(info_msg, 3000)  # 显示3秒
        else:
            QMessageBox.warning(self, "失败", f"电磁阀指令发送失败，返回值: {ret}")
            
    def closeEvent(self, event):
        """窗口关闭事件"""
        try:
            # 停止接收线程
            if self.receiver_thread and self.receiver_thread.isRunning():
                self.stop_receiving(update_ui=False)
            
            # 停止绘图更新线程
            if hasattr(self, 'plot_update_thread') and self.plot_update_thread and self.plot_update_thread.isRunning():
                self.plot_update_thread.stop()
                self.plot_update_thread.wait(2000)
            
            # 关闭设备
            if self.device_handle:
                self.close_device()
            
            event.accept()
        except Exception as e:
            print(f"关闭时出错: {e}")
            event.accept()

    def append_data_point(self, parsed_dict):
        """添加数据点到绘图数据"""
        # 确保绘图数据已初始化
        if not hasattr(self, 'plot_data') or not self.plot_data:
            self.init_plot_data()
            
        # 记录时间
        if self.start_time is None:
            self.start_time = time.time()
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        
        # 添加各个参数的数据
        for param_name in self.plot_data:
            if param_name in parsed_dict:
                self.plot_data[param_name].append(parsed_dict[param_name])
            else:
                # 如果没有新数据，保持上一个值或使用NaN
                if self.plot_data[param_name]:
                    self.plot_data[param_name].append(self.plot_data[param_name][-1])
                else:
                    self.plot_data[param_name].append(np.nan)
        
        # 限制数据点数量，避免内存占用过大
        max_points = 1000
        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            for key in self.plot_data:
                if len(self.plot_data[key]) > max_points:
                    self.plot_data[key] = self.plot_data[key][-max_points:]
                    
def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 使用Fusion样式，看起来更现代
    
    window = CANMonitorEnhancedGUI()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 