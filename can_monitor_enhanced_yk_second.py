"""
CAN通讯协议增强版监控程序
协议要求：
- 波特率：500K
- 扩展帧格式
- 先传高字节，再传低字节（大端序）
- 传输周期：5ms

控制信息：
位移指令：
- CAN ID: 0x01121101
- Byte0: 0xFF表示指令有效，其他表示指令无效
- Byte1~Byte4: 位移指令，无符号整型，0.01mm量纲
"""

import sys
import time
import struct
import os
import numpy as np
import serial
import json
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QTextEdit, QLabel, QComboBox,
                             QSpinBox, QDoubleSpinBox, QGroupBox, QGridLayout, QMessageBox, QTableWidget,
                             QTableWidgetItem, QTabWidget, QSplitter, QScrollArea, QCheckBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QTextCursor, QColor
import pyqtgraph as pg
import zlgcan
import random

class CANDataParser:
    """CAN数据解析器"""
    
    @staticmethod
    def parse_frame_1111201(data):
        """解析第1帧数据 (位移指令和位移反馈)"""
        if len(data) < 8:
            return None
            
        # Byte0~Byte3: 位移指令 (无符号整型，0.01mm)
        displacement_cmd = int.from_bytes(data[0:4], byteorder='big', signed=False)
        
        # Byte4~Byte7: 位移反馈 (无符号整型，0.01mm)
        displacement_feedback = int.from_bytes(data[4:8], byteorder='big', signed=False)
        
        return {
            'displacement_cmd': displacement_cmd * 0.01,      # mm
            'displacement_feedback': displacement_feedback * 0.01  # mm
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
            'pressure1': (4096-pressure1-819.2) * 0.007629,  # Mpa
            'pressure2': (4096-pressure2-819.2) * 0.007629,  # Mpa
            'pressure3': (4096-pressure3-819.2) * 0.007629,  # Mpa
            'pressure4': (4096-pressure4-819.2) * 0.007629  #  Mpa
        }
    
    @staticmethod
    def parse_frame_1111205(data):
        """解析第5帧数据 (报警信息、电磁阀状态、电机转速和压力5)"""
        if len(data) < 8:
            return None
            
        # Byte0: 报警信息
        alarm_byte = data[0]
        alarms = {
            'low_pressure_low': bool(alarm_byte & 0x01),      # Bit0: 低压过低报警 (1故障，0正常)
            'low_pressure_high': bool(alarm_byte & 0x02),     # Bit1: 低压过高报警 (1故障，0正常)
            'displacement_limit': bool(alarm_byte & 0x04),    # Bit2: 位移超限报警 (1故障，0正常)
            'motor_overspeed': bool(alarm_byte & 0x08),       # Bit3: 电机超速报警 (1故障，0正常)
            'motor_overcurrent': bool(alarm_byte & 0x10),     # Bit4: 电机过流报警 (运控自定)
            'igbt_overtemp': bool(alarm_byte & 0x20),         # Bit5: IGBT过温报警 (运控自定)
            'dc_voltage_over': bool(alarm_byte & 0x40),       # Bit6: 母线电压过压报警 (运控自定)
            'dc_voltage_under': bool(alarm_byte & 0x80)       # Bit7: 母线电压欠压报警 (运控自定)
        }
        
        # Byte1: 电磁阀状态和故障信息
        solenoid_byte = data[1]
        solenoid_status = {
            'solenoid0_status': bool(solenoid_byte & 0x01),   # Bit0: 电磁阀0状态 (1打开)
            'solenoid1_status': bool(solenoid_byte & 0x02),   # Bit1: 电磁阀1状态 (1打开)
            'solenoid2_status': bool(solenoid_byte & 0x04),   # Bit2: 电磁阀2状态 (1打开)
            'solenoid0_fault': bool(solenoid_byte & 0x08),    # Bit3: 电磁阀0打开或关闭失败报警 (默认0)
            'solenoid1_fault': bool(solenoid_byte & 0x10),    # Bit4: 电磁阀1打开或关闭失败报警 (默认0)
            'solenoid2_fault': bool(solenoid_byte & 0x20),    # Bit5: 电磁阀2打开或关闭失败报警 (默认0)
            'reserved_bit6': bool(solenoid_byte & 0x40),      # Bit6: 预留
            'reserved_bit7': bool(solenoid_byte & 0x80)       # Bit7: 预留
        }
        
        # Byte2-Byte3: 预留
        reserved_2_3 = data[2:4]
        
        # Byte4-Byte5: 电机转速 (二进制补码，单位1r/min)
        motor_speed_raw = int.from_bytes(data[4:6], byteorder='big', signed=True)
        motor_speed = motor_speed_raw  # 单位：r/min
        
        # Byte6-Byte7: 压力5 (无符号整型，单位0.1Mpa)
        pressure5_raw = int.from_bytes(data[6:8], byteorder='big', signed=False)
        pressure5 = pressure5_raw * 0.1  # 单位：Mpa
        
        return {
            'alarms': alarms,
            'solenoid_status': solenoid_status,
            'reserved_2_3': reserved_2_3,
            'motor_speed': motor_speed,  # r/min
            'pressure5': pressure5      # Mpa
        }

class SerialCommunication:
    """串口通信类"""
    
    def __init__(self, port=10, baudrate=9600, bytesize=8, parity='N', stopbits=1):
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.serial_port = None
        self.is_connected = False
        
    def connect(self):
        """连接串口"""
        try:
            # 将端口号转换为COM端口名称
            com_port = f"COM{self.port}"
            self.serial_port = serial.Serial(
                port=com_port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=1
            )
            self.is_connected = True
            return True
        except Exception as e:
            self.is_connected = False
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.is_connected = False
    
    def send_data(self, data_dict):
        """发送数据到串口"""
        if not self.is_connected or not self.serial_port:
            return False
        
        try:
            # 将数据转换为JSON格式
            json_data = json.dumps(data_dict, ensure_ascii=False)
            # 添加换行符作为数据结束标记
            message = json_data + '\n'
            # 发送数据
            self.serial_port.write(message.encode('utf-8'))
            return True
        except Exception as e:
            return False
    
    def send_motor_data(self, displacement_cmd, displacement_feedback, motor_speed, motor_id_current, 
                       dc_voltage, motor_iq_current, motor_ia_current, motor_ib_current):
        """发送电机相关数据到串口（Arduino格式）"""
        try:
            if not self.is_connected or not self.serial_port:
                return False
            
            # 准备8个float数据
            ch = [
                displacement_cmd,      # 位置指令 (mm)
                displacement_feedback, # 位置反馈 (mm)
                motor_speed,          # 电机转速 (r/min)
                motor_id_current,     # Id电流 (A)
                dc_voltage,           # 母线电压 (V)
                motor_iq_current,     # Iq电流 (A)
                motor_ia_current,     # Ia电流 (A)
                motor_ib_current      # Ib电流 (A)
            ]
            
            # 发送8个float数据
            data_bytes = struct.pack('8f', *ch)
            self.serial_port.write(data_bytes)
            
            # 发送帧尾
            tail = [0x00, 0x00, 0x80, 0x7f]
            self.serial_port.write(bytes(tail))
            
            return True
        except Exception as e:
            return False

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
        data_counter = 0  # 添加数据计数器
        error_count = 0  # 错误计数器
        last_data_time = time.time()  # 记录最后接收数据的时间
        consecutive_no_data_count = 0  # 连续无数据计数
        self.start_time = time.time()  # 记录开始时间
        
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
                        
                        data_counter += 1
                        error_count = 0  # 重置错误计数器
                        last_data_time = time.time()  # 更新最后数据接收时间
                        consecutive_no_data_count = 0  # 重置连续无数据计数
                
                # 检查数据接收状态
                current_time = time.time()
                if current_time - last_data_time > 5:  # 如果超过5秒没有数据
                    consecutive_no_data_count += 1
                    if consecutive_no_data_count > 10:  # 连续10次检查都没有数据
                        if data_counter > 0:  # 之前有数据但现在没有
                            self.raw_data_received.emit(f"警告: 数据接收可能已停止 - 最后数据时间: {current_time - last_data_time:.1f}秒前")
                        consecutive_no_data_count = 0  # 重置计数，避免重复警告              
                
                time.sleep(0.1)  # 降低刷新频率到100ms（10Hz），减少CPU占用
            except Exception as e:
                error_count += 1
                self.raw_data_received.emit(f"接收错误 #{error_count}: {str(e)}")
                # 如果错误次数过多，尝试重新初始化设备
                if error_count > 10:
                    self.raw_data_received.emit("错误次数过多，尝试重新初始化设备...")
                    try:
                        # 尝试重新初始化CAN通道
                        self.zcanlib.ResetCAN(self.chn_handle)
                        time.sleep(0.5)
                        self.zcanlib.StartCAN(self.chn_handle)
                        error_count = 0
                        self.raw_data_received.emit("设备重新初始化成功")
                    except Exception as reset_error:
                        self.raw_data_received.emit(f"设备重新初始化失败: {str(reset_error)}")
                # 不要break，继续运行，避免数据接收完全停止
                time.sleep(0.1)  # 短暂延时后继续
    
    def stop(self):
        self.running = False


class PlotUpdateThread(QThread):
    """绘图更新线程"""
    plot_update_signal = pyqtSignal(str, np.ndarray, np.ndarray)  # param_name, time_array, value_array
    status_update_signal = pyqtSignal(str)  # 状态更新信号
    xrange_update_signal = pyqtSignal(float, float)  # start_time, end_time
    
    def __init__(self, plot_data, time_data, plot_curves, plot_widget, start_time=None):
        super().__init__()
        self.plot_data = plot_data
        self.time_data = time_data
        self.plot_curves = plot_curves
        self.plot_widget = plot_widget
        self.running = False
        self.realtime_enabled = True
        self.pause_enabled = False
        self.update_interval = 100  # 100ms更新间隔 (10Hz)，进一步提高更新频率

        self.last_update_time = 0
        self.update_counter = 0
        # 使用传入的start_time或创建新的时间基准，确保与主线程时间一致
        self.start_time = start_time if start_time is not None else time.time()
        self.last_data_time = 0  # 记录最后数据点的时间
        
    def set_realtime_state(self, enabled):
        """设置实时更新状态"""
        self.realtime_enabled = enabled
        
    def set_pause_state(self, paused):
        """设置暂停状态"""
        self.pause_enabled = paused
        
    def run(self):
        """线程主循环"""
        self.running = True
        self.last_update_time = time.time()
        

        
        while self.running:
            try:
                current_time = time.time()
                
                # 检查是否需要更新
                if (current_time - self.last_update_time >= self.update_interval / 1000.0 and 
                    self.realtime_enabled and not self.pause_enabled):
                    
                    # 更新所有曲线
                    self.update_all_curves()
                    
                    # 更新X轴范围
                    self.update_x_range()
                                                          
                    self.last_update_time = current_time
                    self.update_counter += 1
                    

                
                # 短暂休眠，避免CPU占用过高
                time.sleep(0.01)  # 10ms
                
            except Exception as e:
                self.status_update_signal.emit(f"绘图更新错误: {str(e)}")
                time.sleep(0.1)  # 错误时稍长休眠
                
    def update_all_curves(self):
        """更新所有曲线"""
        try:
            # 优化：只获取数据引用，避免大量数据复制
            if not self.time_data or len(self.time_data) == 0:
                return
                
            # 检查数据是否足够进行绘图
            if len(self.time_data) < 2:
                return
            
            # 使用相对时间
            current_time = time.time() - self.start_time
            self.last_data_time = self.time_data[-1] if self.time_data else 0
            
            # 转换为numpy数组以提高性能
            time_array = np.array(self.time_data)
            
            # 添加调试信息
            if not hasattr(self, '_curve_debug_counter'):
                self._curve_debug_counter = 0
            self._curve_debug_counter += 1
            
           
            # 更新每个参数的曲线
            for param_name in self.plot_data:
                if param_name in self.plot_curves and len(self.plot_data[param_name]) > 0:
                    # 确保数据长度一致
                    data_length = min(len(time_array), len(self.plot_data[param_name]))
                    if data_length > 0:
                        # 转换为numpy数组
                        value_array = np.array(self.plot_data[param_name][:data_length])
                        
                        # 简化的数据采样：只在数据量很大时进行轻微下采样
                        if data_length > 3600:  # 数据量很大时的采样（60分钟以上）
                            # 轻微下采样，保留更多数据点
                            sample_interval = max(1, data_length // 3600)  # 采样到3600个点（60分钟）
                            time_subset = time_array[::sample_interval][:3600]
                            value_array = value_array[::sample_interval][:3600]
                        else:
                            # 直接使用原始时间数据，不进行下采样
                            time_subset = time_array[:data_length]
                        
                        # 验证数据有效性
                        if len(time_subset) > 0 and len(value_array) > 0:
                            # 检查时间数据是否合理
                            if np.any(time_subset >= 0) and np.any(~np.isnan(value_array)):
                                # 检查数据长度是否合理（避免数据不一致）
                                if len(time_subset) == len(value_array) and len(time_subset) >= 2:
                                    # 发送更新信号
                                    self.plot_update_signal.emit(param_name, time_subset, value_array)
                                else:
                                    pass

                        
        except Exception as e:
            self.status_update_signal.emit(f"更新曲线错误: {str(e)}")
            
    def update_x_range(self):
        """更新X轴范围，实现滚动效果"""
        try:
            if not self.time_data or len(self.time_data) < 2:
                return
                
            # 基于数据的实际时间范围计算显示窗口
            data_start_time = self.time_data[0]
            data_end_time = self.time_data[-1]
            data_time_span = data_end_time - data_start_time
            
            # 设置显示窗口大小（秒）
            window_size = 900.0  # 15分钟显示窗口，确保能够显示足够长时间的数据
            
            # 计算显示范围 - 基于数据时间范围
            if data_time_span > window_size:
                # 当数据时间跨度超过窗口大小时，显示最新的window_size秒数据
                start_time = data_end_time - window_size
                end_time = data_end_time + 1.0  # 额外显示1秒作为缓冲
            else:
                # 当数据时间跨度不足窗口大小时，显示全部数据并添加缓冲
                start_time = data_start_time - 1.0  # 向前缓冲1秒
                end_time = data_end_time + 1.0  # 向后缓冲1秒
            
            # 确保时间范围有效
            if start_time < 0:
                start_time = 0
            if end_time <= start_time:
                end_time = start_time + 1.0
            
            # 发送X轴范围更新信号
            self.xrange_update_signal.emit(start_time, end_time)
            
            # 强制更新所有曲线数据，确保与X轴范围同步
            self.update_all_curves()
            

            
        except Exception as e:
            self.status_update_signal.emit(f"更新X轴范围错误: {str(e)}")
            
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
        
        # 初始化串口通信
        self.serial_comm = SerialCommunication(port=11, baudrate=9600, bytesize=8, parity='N', stopbits=1)
        
        # 初始化绘图更新暂停标志
        self.pause_plot_updates = False
        
        # 数据更新计数器，用于控制更新频率
        self.update_counter = 0
        
        # 初始化绘图数据存储（在UI创建之前）
        self.time_data = []
        self.plot_data = {}
        self.plot_curves = {}
        self._start_time = time.time()  # 用于性能监控的程序启动时间
        self._plot_update_counter = 0
        
        # 初始化UI
        self.init_ui()
        
        # 创建定时器检查绘图线程状态
        self.plot_status_timer = QTimer()
        self.plot_status_timer.timeout.connect(self.auto_check_plot_thread)
        self.plot_status_timer.start(3000)  # 每3秒检查一次，更频繁的检查
        
        # 初始化自动保存相关变量
        self.auto_save_enabled = False
        self.auto_save_timer = None
        self.last_save_time = 0
        self.save_counter = 0
        
        # 初始化手动数据记录相关变量
        self.data_recording_enabled = False
        self.recorded_time_data = []
        self.recorded_plot_data = {}
        self.recording_start_time = 0
        
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
        """
        创建设备控制组
        根据协议要求：波特率500K，扩展帧格式
        """
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
        
        # 波特率选择（协议要求：500K）
        layout.addWidget(QLabel("波特率:"), 1, 2)
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems([
            "100000", "250000", "500000", "1000000"
        ])
        self.baudrate_combo.setCurrentText("500000")  # 默认500K，符合协议要求
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
        
        # 串口连接控制
        layout.addWidget(QLabel("串口通信:"), 3, 0)
        self.serial_connect_btn = QPushButton("连接串口")
        self.serial_connect_btn.clicked.connect(self.connect_serial)
        layout.addWidget(self.serial_connect_btn, 3, 1)
        
        self.serial_disconnect_btn = QPushButton("断开串口")
        self.serial_disconnect_btn.clicked.connect(self.disconnect_serial)
        self.serial_disconnect_btn.setEnabled(False)
        layout.addWidget(self.serial_disconnect_btn, 3, 2)
        
        # 串口状态显示
        self.serial_status_label = QLabel("串口状态: 未连接")
        layout.addWidget(self.serial_status_label, 3, 3)
        
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
        self.cmd_value_input.setRange(0, 1500)  # 位移指令范围：0-1500mm
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
        
        # 正弦指令控制组
        sine_group = QGroupBox("正弦位移指令")
        sine_layout = QHBoxLayout()  # 改为水平布局
        sine_layout.setSpacing(20)  # 进一步增加控件间距
        
        # 正弦指令参数 - 单行布局
        sine_layout.addWidget(QLabel("中心:"))
        self.sine_center_spin = QDoubleSpinBox()
        self.sine_center_spin.setRange(0, 1500)  # 中心位移范围：0-1500mm
        self.sine_center_spin.setValue(750)  # 默认中心位移750mm（1500/2）
        self.sine_center_spin.setSuffix(" mm")
        self.sine_center_spin.setDecimals(1)
        self.sine_center_spin.setMinimumWidth(120)  # 进一步增加最小宽度
        self.sine_center_spin.setMaximumWidth(150)  # 进一步增加最大宽度
        sine_layout.addWidget(self.sine_center_spin)
        
        sine_layout.addWidget(QLabel("振幅:"))
        self.sine_amplitude_spin = QDoubleSpinBox()
        self.sine_amplitude_spin.setRange(0, 750)  # 振幅范围：0-750mm（确保中心±振幅不超过1500）
        self.sine_amplitude_spin.setValue(200)  # 默认振幅200mm
        self.sine_amplitude_spin.setSuffix(" mm")
        self.sine_amplitude_spin.setDecimals(1)
        self.sine_amplitude_spin.setMinimumWidth(120)  # 进一步增加最小宽度
        self.sine_amplitude_spin.setMaximumWidth(150)  # 进一步增加最大宽度
        sine_layout.addWidget(self.sine_amplitude_spin)
        
        sine_layout.addWidget(QLabel("频率:"))
        self.sine_frequency_spin = QDoubleSpinBox()
        self.sine_frequency_spin.setRange(0.01, 10.0)
        self.sine_frequency_spin.setValue(0.1)  # 默认频率0.1Hz
        self.sine_frequency_spin.setSuffix(" Hz")
        self.sine_frequency_spin.setDecimals(2)
        self.sine_frequency_spin.setMinimumWidth(120)  # 进一步增加最小宽度
        self.sine_frequency_spin.setMaximumWidth(150)  # 进一步增加最大宽度
        sine_layout.addWidget(self.sine_frequency_spin)
        
        # 正弦指令使能控制
        self.sine_enable_checkbox = QCheckBox("使能")
        self.sine_enable_checkbox.setChecked(True)
        sine_layout.addWidget(self.sine_enable_checkbox)
        
        # 开始/停止正弦指令按钮
        self.start_sine_btn = QPushButton("开始")
        self.start_sine_btn.clicked.connect(self.start_sine_command)
        self.start_sine_btn.setMinimumWidth(100)  # 进一步增加按钮最小宽度
        self.start_sine_btn.setMaximumWidth(120)  # 进一步增加按钮最大宽度
        sine_layout.addWidget(self.start_sine_btn)
        
        # 停止正弦指令按钮
        self.stop_sine_btn = QPushButton("停止")
        self.stop_sine_btn.clicked.connect(self.stop_sine_command)
        self.stop_sine_btn.setEnabled(False)
        self.stop_sine_btn.setMinimumWidth(100)  # 进一步增加按钮最小宽度
        self.stop_sine_btn.setMaximumWidth(120)  # 进一步增加按钮最大宽度
        sine_layout.addWidget(self.stop_sine_btn)
        
        sine_layout.addStretch()  # 添加弹性空间
        
        sine_group.setLayout(sine_layout)
        parent_layout.addWidget(sine_group)
        
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
        
        # 初始化正弦指令相关变量
        self.sine_timer = None
        self.sine_start_time = None
        
    def on_cmd_type_changed(self, cmd_type):
        """指令类型改变时的处理"""
        if cmd_type == "位移指令":
            self.unit_label.setText("mm")
            self.cmd_value_input.setRange(0, 1500)  # 位移范围：0-1500mm
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
        
        # 曲线采集控制组
        data_control_group = QGroupBox("数据采集控制")
        data_control_layout = QHBoxLayout()
        
        # 暂停数据采集按钮
        self.pause_data_btn = QPushButton("暂停采集")
        self.pause_data_btn.clicked.connect(self.pause_data_collection)
        self.pause_data_btn.setEnabled(False)  # 初始状态禁用
        data_control_layout.addWidget(self.pause_data_btn)
        
        # 重新开始数据采集按钮
        self.restart_data_btn = QPushButton("重新开始")
        self.restart_data_btn.clicked.connect(self.restart_data_collection)
        self.restart_data_btn.setEnabled(False)  # 初始状态禁用
        data_control_layout.addWidget(self.restart_data_btn)
        
        data_control_group.setLayout(data_control_layout)
        layout.addWidget(data_control_group)
        
        # 曲线选择区域（用QScrollArea包裹）
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(2)
        
        # 添加一键选择控制按钮
        select_control_layout = QHBoxLayout()
        select_control_layout.setSpacing(10)  # 增加按钮间距
        
        # 一键选中按钮
        self.select_all_btn = QPushButton("全选")
        self.select_all_btn.clicked.connect(self.select_all_curves)
        self.select_all_btn.setMinimumWidth(80)  # 增加最小宽度
        self.select_all_btn.setMaximumWidth(100)  # 增加最大宽度
        select_control_layout.addWidget(self.select_all_btn)
        
        # 一键全不选按钮
        self.deselect_all_btn = QPushButton("全不选")
        self.deselect_all_btn.clicked.connect(self.deselect_all_curves)
        self.deselect_all_btn.setMinimumWidth(80)  # 增加最小宽度
        self.deselect_all_btn.setMaximumWidth(100)  # 增加最大宽度
        select_control_layout.addWidget(self.deselect_all_btn)
        
        select_control_layout.addStretch()  # 添加弹性空间
        scroll_layout.addLayout(select_control_layout)
        
        # 曲线颜色映射（与init_plot_curves一致）
        curve_colors = {
            "displacement_cmd": "#ff0000",   # 红色
            "displacement_feedback": "#00ff00", # 绿色
            "motor_speed": "#0000ff",         # 蓝色
            "pressure5": "#00ffff",           # 青色
            "dc_voltage": "#ff00ff",          # 洋红
            "motor_id_current": "#ffff00",    # 黄色
            "motor_iq_current": "#000000",    # 黑色
            "igbt_temp": "#ffa500",           # 橙色
            "motor_ia_current": "#800080",    # 紫色
            "motor_ib_current": "#a52a2a",    # 棕色
            "pt100_a": "#ffc0cb",             # 粉色
            "pt100_b": "#808080",             # 灰色
            "pt100_c": "#808000",             # 橄榄色
            "pressure1": "#000080",           # 海军蓝
            "pressure2": "#008080",           # 青绿色
            "pressure3": "#800000",           # 栗色
            "pressure4": "#00ff00",           # 酸橙色
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
        
        # 设置现代化的外观
        self.plot_widget.setBackground('#f8f9fa')  # 浅灰色背景
        self.plot_widget.showGrid(x=True, y=True, alpha=0.15)  # 更淡的网格
        
        # 设置标签
        self.plot_widget.setLabel('left', '数值', color='#2c3e50', size='12pt')
        self.plot_widget.setLabel('bottom', '时间 (s)', color='#2c3e50', size='12pt')
        
        # 设置轴的颜色和样式
        self.plot_widget.getAxis('left').setPen(pg.mkPen(color='#34495e', width=1))
        self.plot_widget.getAxis('bottom').setPen(pg.mkPen(color='#34495e', width=1))
        
        # 设置初始X轴范围
        self.plot_widget.setXRange(0, 5)
        
        # 配置轴的行为 - 优化滚动性能
        self.plot_widget.disableAutoRange(axis='x')  # 禁用X轴自动范围
        self.plot_widget.enableAutoRange(axis='y')   # 启用Y轴自动范围
        self.plot_widget.setLimits(xMin=0, xMax=None)
        
        # 配置ViewBox - 优化滚动性能
        view_box = self.plot_widget.getViewBox()
        view_box.setMouseEnabled(x=False, y=True)  # 禁用X轴鼠标操作，避免用户干扰
        view_box.setAutoVisible(x=False)           # 禁用X轴自动可见
        view_box.setAspectLocked(False)           # 禁用宽高比锁定
        view_box.setMenuEnabled(False)            # 禁用右键菜单
        view_box.setAutoPan(x=False, y=False)     # 禁用自动平移
        view_box.enableAutoRange(axis='x', enable=False)  # 禁用X轴自动范围
        
        # 性能优化设置
        self.plot_widget.setDownsampling(auto=True, mode='peak')  # 使用峰值下采样
        self.plot_widget.setClipToView(True)      # 裁剪到视图
        self.plot_widget.setAntialiasing(True)    # 抗锯齿
        
        # 添加图例
        self.legend = self.plot_widget.addLegend(offset=(10, 10))
        self.legend.setBrush(pg.mkBrush(color='#ffffff', alpha=0.8))
        self.legend.setPen(pg.mkPen(color='#bdc3c7', width=1))
        
        # 初始化绘图数据
        self.init_plot_data()
        
        # 初始化绘图曲线
        self.init_plot_curves()
        
        # 优化绘图性能
        self.optimize_plot_performance()
        
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
        # 定义现代化的曲线颜色映射
        curve_colors = {
            "displacement_cmd": "#e74c3c",      # 红色
            "displacement_feedback": "#27ae60",  # 绿色
            "motor_speed": "#3498db",           # 蓝色
            "pressure5": "#1abc9c",             # 青色
            "dc_voltage": "#f39c12",            # 橙色
            "motor_id_current": "#9b59b6",      # 紫色
            "motor_iq_current": "#34495e",      # 深灰色
            "igbt_temp": "#e67e22",             # 深橙色
            "motor_ia_current": "#8e44ad",      # 深紫色
            "motor_ib_current": "#d35400",      # 棕色
            "pt100_a": "#e91e63",               # 粉色
            "pt100_b": "#795548",               # 棕色
            "pt100_c": "#4caf50",               # 绿色
            "pressure1": "#2196f3",             # 蓝色
            "pressure2": "#00bcd4",             # 青色
            "pressure3": "#ff5722",             # 红色
            "pressure4": "#4caf50",             # 绿色
        }
        
        param_names = list(self.plot_data.keys())
        
        # 定义参数的中文名称映射
        param_names_cn = {
            "displacement_cmd": "位移指令",
            "displacement_feedback": "位移反馈", 
            "motor_speed": "电机转速",
            "pressure5": "压力5",
            "dc_voltage": "DC母线电压",
            "motor_id_current": "电机Id电流",
            "motor_iq_current": "电机Iq电流",
            "igbt_temp": "IGBT温度",
            "motor_ia_current": "电机Ia电流",
            "motor_ib_current": "电机Ib电流",
            "pt100_a": "PT100-A",
            "pt100_b": "PT100-B",
            "pt100_c": "PT100-C",
            "pressure1": "压力1",
            "pressure2": "压力2",
            "pressure3": "压力3",
            "pressure4": "压力4",
        }
        
        for param_name in param_names:
            color = curve_colors.get(param_name, '#2c3e50')  # 默认深灰色
            # 设置现代化的曲线样式
            pen = pg.mkPen(color=color, width=2.5, style=Qt.SolidLine)
            
            # 获取中文名称
            display_name = param_names_cn.get(param_name, param_name)
            curve = self.plot_widget.plot(pen=pen, name=display_name)
            
            # 设置曲线的性能优化
            curve.setDownsampling(auto=True)
            
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
                
    def select_all_curves(self):
        """一键选中所有曲线"""
        for param_name, checkbox in self.curve_checkboxes.items():
            checkbox.setChecked(True)
            if param_name in self.plot_curves:
                self.plot_curves[param_name].show()
        self.statusBar().showMessage("已选中所有曲线", 2000)
        
    def deselect_all_curves(self):
        """一键全不选所有曲线"""
        for param_name, checkbox in self.curve_checkboxes.items():
            checkbox.setChecked(False)
            if param_name in self.plot_curves:
                self.plot_curves[param_name].hide()
        self.statusBar().showMessage("已取消选中所有曲线", 2000)
                
    def pause_data_collection(self):
        """暂停数据采集"""
        if hasattr(self, 'receiver_thread') and self.receiver_thread and self.receiver_thread.isRunning():
            # 暂停接收线程
            self.receiver_thread.stop()
            self.receiver_thread.wait()
            self.receiver_thread = None
            
            # 更新UI状态
            self.pause_data_btn.setEnabled(False)
            self.restart_data_btn.setEnabled(True)
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            
            self.statusBar().showMessage("数据采集已暂停")
            
    def restart_data_collection(self):
        """重新开始数据采集"""
        if not self.chn_handle:
            QMessageBox.warning(self, "警告", "请先打开设备!")
            return
            
        try:
            # 重新启动接收线程
            self.receiver_thread = CANReceiverThread(self.zcanlib, self.chn_handle)
            self.receiver_thread.data_received.connect(self.on_data_received)
            self.receiver_thread.start()
            
            # 更新UI状态
            self.pause_data_btn.setEnabled(True)
            self.restart_data_btn.setEnabled(False)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            
            self.statusBar().showMessage("数据采集已重新开始")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"重新启动接收线程失败: {str(e)}")
                
    def start_plot_update_thread(self):
        """启动绘图更新线程"""
        # 确保绘图数据已初始化
        if not hasattr(self, 'plot_data') or not self.plot_data:
            self.init_plot_data()
            
        # 确保绘图曲线已初始化
        if not hasattr(self, 'plot_curves') or not self.plot_curves:
            self.init_plot_curves()
            
        # 如果线程不存在或已停止，创建新线程
        if not hasattr(self, 'plot_update_thread') or not self.plot_update_thread or not self.plot_update_thread.isRunning():
            # 如果旧线程存在，先停止它
            if hasattr(self, 'plot_update_thread') and self.plot_update_thread:
                self.plot_update_thread.stop()
                self.plot_update_thread.wait(1000)  # 等待最多1秒
                
            # 创建新线程，使用相对时间基准
            self.plot_update_thread = PlotUpdateThread(
                self.plot_data, self.time_data, self.plot_curves, self.plot_widget, self._start_time
            )
            self.plot_update_thread.plot_update_signal.connect(self.update_plot_curve)
            self.plot_update_thread.status_update_signal.connect(self.statusBar().showMessage)
            self.plot_update_thread.xrange_update_signal.connect(self.update_plot_xrange)
            self.plot_update_thread.start()
            

            
    def update_plot_curve(self, param_name, time_array, value_array):
        """更新绘图曲线"""
        try:
                
            # 检查参数是否存在于曲线字典中
            if param_name not in self.plot_curves:
                return
                
            # 获取对应的曲线对象
            curve = self.plot_curves[param_name]
            if curve is None:
                return
                
            # 验证数据有效性
            if len(time_array) == 0 or len(value_array) == 0:
                return
                
            # 确保时间数组和数值数组长度一致
            if len(time_array) != len(value_array):
                # 取较短的长度
                min_length = min(len(time_array), len(value_array))
                time_array = time_array[:min_length]
                value_array = value_array[:min_length]
                
            # 过滤无效数据（NaN和无穷大）
            valid_mask = ~(np.isnan(value_array) | np.isinf(value_array))
            if not np.any(valid_mask):
                return
                
            # 应用有效数据掩码
            valid_time = time_array[valid_mask]
            valid_values = value_array[valid_mask]
            
            # 检查过滤后的数据是否为空
            if len(valid_time) == 0 or len(valid_values) == 0:
                return
                
            # 检查曲线是否可见
            if not curve.isVisible():
                return
                
            # 更新曲线数据
            curve.setData(valid_time, valid_values)
            
            # 增加更新计数器
            self._plot_update_counter += 1
            
            # 性能优化：减少强制重绘频率
            if self._plot_update_counter % 20 == 0:  # 每20次更新强制重绘一次
                self.plot_widget.update()
                
        except Exception as e:
            # 错误处理，避免绘图崩溃
            error_msg = f"更新曲线 {param_name} 时发生错误: {str(e)}"
            self.statusBar().showMessage(error_msg)
            
    def update_plot_xrange(self, start_time, end_time):
        """更新绘图X轴范围，实现滚动效果"""
        try:
            # 验证时间范围的有效性
            if start_time < 0:
                start_time = 0
            if end_time <= start_time:
                end_time = start_time + 1.0
                
            # 设置X轴范围
            self.plot_widget.setXRange(start_time, end_time)
            
            # 性能优化：限制更新频率
            if not hasattr(self, '_xrange_update_counter'):
                self._xrange_update_counter = 0
            self._xrange_update_counter += 1
            

                
        except Exception as e:
            # 错误处理
            error_msg = f"更新X轴范围时发生错误: {str(e)}"
            self.statusBar().showMessage(error_msg)
            
    def optimize_plot_performance(self):
        """优化绘图性能"""
        try:
            # 设置绘图性能参数
            if hasattr(self, 'plot_widget') and self.plot_widget:
                # 启用抗锯齿
                self.plot_widget.setAntialiasing(True)
                
                # 设置下采样模式
                self.plot_widget.setDownsampling(auto=True, mode='peak')
                
                # 设置裁剪到视图
                self.plot_widget.setClipToView(True)
                
                # 禁用自动范围以避免性能问题
                self.plot_widget.disableAutoRange(axis='x')
                self.plot_widget.enableAutoRange(axis='y')
                
                # 设置视图框参数
                view_box = self.plot_widget.getViewBox()
                if view_box:
                    view_box.setMouseEnabled(x=False, y=True)
                    view_box.setAutoVisible(x=False)
                    view_box.setAspectLocked(False)
                    view_box.setMenuEnabled(False)
                    view_box.setAutoPan(x=False, y=False)
                    
        except Exception as e:
            pass
            
    def batch_update_curves(self, update_data):
        """批量更新曲线数据，提高性能"""
        try:
            # 暂停绘图更新以提高性能
            self.plot_widget.setUpdatesEnabled(False)
            
            # 批量更新所有曲线
            for param_name, (time_array, value_array) in update_data.items():
                if param_name in self.plot_curves:
                    curve = self.plot_curves[param_name]
                    if curve and curve.isVisible():
                        # 验证数据有效性
                        if len(time_array) > 0 and len(value_array) > 0:
                            # 过滤无效数据
                            valid_mask = ~(np.isnan(value_array) | np.isinf(value_array))
                            if np.any(valid_mask):
                                valid_time = time_array[valid_mask]
                                valid_values = value_array[valid_mask]
                                if len(valid_time) > 0 and len(valid_values) > 0:
                                    curve.setData(valid_time, valid_values)
            
            # 恢复绘图更新
            self.plot_widget.setUpdatesEnabled(True)
            
            # 强制重绘
            self.plot_widget.update()
            
        except Exception as e:
            # 确保恢复绘图更新
            self.plot_widget.setUpdatesEnabled(True)
            
    def monitor_plot_performance(self):
        """监控绘图性能"""
        try:
            import psutil
            import os
            
            # 获取当前进程的内存使用情况
            process = psutil.Process(os.getpid())
            memory_info = process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024
            
            # 获取数据点数量
            data_points = len(self.time_data) if hasattr(self, 'time_data') else 0
            
            # 计算更新频率 - 使用程序运行时间
            if hasattr(self, '_plot_update_counter') and hasattr(self, '_start_time'):
                update_rate = self._plot_update_counter / max(1, (time.time() - self._start_time))
            else:
                update_rate = 0
                
            # 更新状态栏显示性能信息
            performance_info = f"内存: {memory_mb:.1f}MB | 数据点: {data_points} | 更新频率: {update_rate:.1f}Hz"
            self.statusBar().showMessage(performance_info)
            

                
        except ImportError:
            # 如果没有psutil库，跳过内存监控
            pass
        except Exception as e:
            pass
            
    def check_plot_thread_status(self):
        """检查绘图更新线程状态"""
        try:
            if hasattr(self, 'plot_update_thread') and self.plot_update_thread:
                if self.plot_update_thread.isRunning():
                    # 检查线程是否正常更新
                    if hasattr(self.plot_update_thread, 'update_counter') and hasattr(self, '_start_time'):
                        update_rate = self.plot_update_thread.update_counter / max(1, (time.time() - self._start_time))
                        self.statusBar().showMessage(f"绘图线程运行中 - 更新频率: {update_rate:.1f}Hz")
                        

                    else:
                        self.statusBar().showMessage("绘图线程运行中 - 无更新计数")
                else:
                    self.statusBar().showMessage("绘图线程已停止 - 尝试重启")
                    # 尝试重启绘图线程
                    self.start_plot_update_thread()
            else:
                self.statusBar().showMessage("绘图线程未启动 - 正在启动")
                self.start_plot_update_thread()
                
        except Exception as e:
            self.statusBar().showMessage(f"检查绘图线程状态时发生错误: {str(e)}")
            
    def auto_check_plot_thread(self):
        """自动检查绘图线程状态并恢复"""
        try:
            if hasattr(self, 'plot_update_thread') and self.plot_update_thread:
                if self.plot_update_thread.isRunning():
                    pass
                else:
                    self.statusBar().showMessage("绘图线程已停止，自动重启中...")
                    self.start_plot_update_thread()
            else:
                self.statusBar().showMessage("绘图线程未启动，正在启动...")
                self.start_plot_update_thread()
                
        except Exception as e:
            self.statusBar().showMessage(f"自动检查绘图线程时发生错误: {str(e)}")
            
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
        self.save_btn = QPushButton("开始记录")
        self.save_btn.clicked.connect(self.toggle_data_recording)
        button_layout.addWidget(self.save_btn)

        # 图例显示控制按钮
        self.legend_btn = QPushButton("显示图例")
        self.legend_btn.setCheckable(True)
        self.legend_btn.setChecked(True)
        self.legend_btn.clicked.connect(self.toggle_legend)
        button_layout.addWidget(self.legend_btn)
        
        # 自动缩放控制按钮
        self.auto_scale_btn = QPushButton("自动缩放")
        self.auto_scale_btn.setCheckable(True)
        self.auto_scale_btn.setChecked(True)
        self.auto_scale_btn.clicked.connect(self.toggle_auto_scale)
        button_layout.addWidget(self.auto_scale_btn)
        
        # 自动保存控制按钮
        self.auto_save_btn = QPushButton("自动保存")
        self.auto_save_btn.setCheckable(True)
        self.auto_save_btn.setChecked(False)
        self.auto_save_btn.clicked.connect(self.toggle_auto_save)
        button_layout.addWidget(self.auto_save_btn)
        
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
            # 启动绘图更新线程
            self.start_plot_update_thread()
            
            # 启动接收线程
            self.receiver_thread = CANReceiverThread(self.zcanlib, self.chn_handle)
            self.receiver_thread.data_received.connect(self.on_data_received)
            self.receiver_thread.start()
            
            # 更新UI状态
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.pause_data_btn.setEnabled(True)
            self.restart_data_btn.setEnabled(False)
            
            self.statusBar().showMessage("开始接收CAN数据...")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"启动接收线程失败: {str(e)}")
    

    
    def on_stop_button_clicked(self):
        """停止接收按钮点击处理"""
        self.stop_receiving(update_ui=True)
        
    def stop_receiving(self, update_ui=True):
        """停止接收数据"""
        try:
            # 停止接收线程
            if self.receiver_thread and self.receiver_thread.isRunning():
                self.receiver_thread.stop()
                self.receiver_thread.wait(1000)  # 等待最多1秒
                
            # 停止绘图更新线程
            if self.plot_update_thread and self.plot_update_thread.isRunning():
                self.plot_update_thread.stop()
                self.plot_update_thread.wait(1000)  # 等待最多1秒
                
            if update_ui:
                # 更新UI状态
                self.start_btn.setEnabled(True)
                self.stop_btn.setEnabled(False)
                self.pause_data_btn.setEnabled(False)
                self.restart_data_btn.setEnabled(False)
                
                self.statusBar().showMessage("数据接收已停止")
                
        except Exception as e:
            if update_ui:
                QMessageBox.critical(self, "错误", f"停止接收线程失败: {str(e)}")
                
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
        """更新第1帧数据 (位移指令和位移反馈)"""
        parsed = self.parser.parse_frame_1111201(data)
        if parsed:
            # 立即更新UI显示，提高数据可见性
            if 'displacement_cmd' in self.data_labels:
                self.data_labels['displacement_cmd']['label'].setText(f"位移指令: {parsed['displacement_cmd']:.2f} mm")
            if 'displacement_feedback' in self.data_labels:
                self.data_labels['displacement_feedback']['label'].setText(f"位移反馈: {parsed['displacement_feedback']:.2f} mm")
            # 注意：第1帧不包含电机转速和压力5，这些数据在第5帧中
            
            # 发送数据到串口
            if self.serial_comm.is_connected:
                # 第1帧只包含位移指令和位移反馈，其他数据等后续帧
                self.serial_comm.send_motor_data(
                    displacement_cmd=parsed['displacement_cmd'],
                    displacement_feedback=parsed['displacement_feedback'],
                    motor_speed=0.0,       # 暂时设为0，等第5帧数据
                    motor_id_current=0.0,  # 暂时设为0，等第2帧数据
                    dc_voltage=0.0,        # 暂时设为0，等第2帧数据
                    motor_iq_current=0.0,  # 暂时设为0，等第2帧数据
                    motor_ia_current=0.0,  # 暂时设为0，等第3帧数据
                    motor_ib_current=0.0   # 暂时设为0，等第3帧数据
                )
            
            self.append_data_point(parsed)

            
    def update_frame_1111202(self, data):
        """更新第2帧数据"""
        parsed = self.parser.parse_frame_1111202(data)
        if parsed:
            # 立即更新UI显示，提高数据可见性
            if 'dc_voltage' in self.data_labels:
                self.data_labels['dc_voltage']['label'].setText(f"DC母线电压: {parsed['dc_voltage']} V")
            if 'motor_id_current' in self.data_labels:
                self.data_labels['motor_id_current']['label'].setText(f"电机Id电流: {parsed['motor_id_current']:.1f} A")
            if 'motor_iq_current' in self.data_labels:
                self.data_labels['motor_iq_current']['label'].setText(f"电机Iq电流: {parsed['motor_iq_current']:.1f} A")
            if 'igbt_temp' in self.data_labels:
                self.data_labels['igbt_temp']['label'].setText(f"IGBT温度: {parsed['igbt_temp']} °C")
            
            # 发送完整数据到串口（包含Id电流和Iq电流）
            if self.serial_comm.is_connected:
                # 获取最新的第1帧数据
                latest_frame1_data = self.get_latest_frame1_data()
                if latest_frame1_data:
                    self.serial_comm.send_motor_data(
                        displacement_cmd=latest_frame1_data.get('displacement_cmd', 0.0),
                        displacement_feedback=latest_frame1_data.get('displacement_feedback', 0.0),
                        motor_speed=0.0,       # 暂时设为0，等第5帧数据
                        motor_id_current=parsed['motor_id_current'],
                        dc_voltage=parsed['dc_voltage'],
                        motor_iq_current=parsed['motor_iq_current'],
                        motor_ia_current=0.0,  # 暂时设为0，等第3帧数据
                        motor_ib_current=0.0   # 暂时设为0，等第3帧数据
                    )
            
            self.append_data_point(parsed)
    
    def get_latest_frame1_data(self):
        """获取最新的第1帧数据 (位移指令和位移反馈)"""
        # 从绘图数据中获取最新的第1帧数据
        if 'displacement_cmd' in self.plot_data and len(self.plot_data['displacement_cmd']) > 0:
            latest_index = len(self.plot_data['displacement_cmd']) - 1
            return {
                'displacement_cmd': self.plot_data['displacement_cmd'][latest_index],
                'displacement_feedback': self.plot_data['displacement_feedback'][latest_index] if 'displacement_feedback' in self.plot_data and len(self.plot_data['displacement_feedback']) > latest_index else 0.0
                # 注意：电机转速现在在第5帧中，不在这里获取
            }
        return None
    
    def get_latest_frame2_data(self):
        """获取最新的第2帧数据"""
        # 从绘图数据中获取最新的第2帧数据
        if 'dc_voltage' in self.plot_data and len(self.plot_data['dc_voltage']) > 0:
            latest_index = len(self.plot_data['dc_voltage']) - 1
            return {
                'dc_voltage': self.plot_data['dc_voltage'][latest_index],
                'motor_id_current': self.plot_data['motor_id_current'][latest_index] if 'motor_id_current' in self.plot_data and len(self.plot_data['motor_id_current']) > latest_index else 0.0,
                'motor_iq_current': self.plot_data['motor_iq_current'][latest_index] if 'motor_iq_current' in self.plot_data and len(self.plot_data['motor_iq_current']) > latest_index else 0.0
            }
        return None
    
    def get_latest_frame3_data(self):
        """获取最新的第3帧数据 (电机Ia/Ib电流)"""
        # 从绘图数据中获取最新的第3帧数据
        if 'motor_ia_current' in self.plot_data and len(self.plot_data['motor_ia_current']) > 0:
            latest_index = len(self.plot_data['motor_ia_current']) - 1
            return {
                'motor_ia_current': self.plot_data['motor_ia_current'][latest_index],
                'motor_ib_current': self.plot_data['motor_ib_current'][latest_index] if 'motor_ib_current' in self.plot_data and len(self.plot_data['motor_ib_current']) > latest_index else 0.0
            }
        return None
            
    def update_frame_1111203(self, data):
        """更新第3帧数据"""
        parsed = self.parser.parse_frame_1111203(data)
        if parsed:
            # 立即更新UI显示，提高数据可见性
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
            
            # 发送完整数据到串口（包含所有电机数据）
            if self.serial_comm.is_connected:
                # 获取最新的第1帧和第2帧数据
                latest_frame1_data = self.get_latest_frame1_data()
                latest_frame2_data = self.get_latest_frame2_data()
                
                if latest_frame1_data and latest_frame2_data:
                    self.serial_comm.send_motor_data(
                        displacement_cmd=latest_frame1_data.get('displacement_cmd', 0.0),
                        displacement_feedback=latest_frame1_data.get('displacement_feedback', 0.0),
                        motor_speed=0.0,       # 暂时设为0，等第5帧数据
                        motor_id_current=latest_frame2_data.get('motor_id_current', 0.0),
                        dc_voltage=latest_frame2_data.get('dc_voltage', 0.0),
                        motor_iq_current=latest_frame2_data.get('motor_iq_current', 0.0),
                        motor_ia_current=parsed['motor_ia_current'],
                        motor_ib_current=parsed['motor_ib_current']
                    )
            
            self.append_data_point(parsed)
            
    def update_frame_1111204(self, data):
        """更新第4帧数据"""
        parsed = self.parser.parse_frame_1111204(data)
        if parsed:
            # 立即更新UI显示，提高数据可见性
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
        """更新第5帧数据 (报警信息、电磁阀状态、电机转速和压力5)"""
        parsed = self.parser.parse_frame_1111205(data)
        if parsed:
            # 更新电机转速和压力5显示
            if 'motor_speed' in self.data_labels:
                self.data_labels['motor_speed']['label'].setText(f"电机转速: {parsed.get('motor_speed', 0)} r/min")
            if 'pressure5' in self.data_labels:
                self.data_labels['pressure5']['label'].setText(f"压力5: {parsed.get('pressure5', 0):.1f} Mpa")
            
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
            
            # 发送包含电机转速和压力5的完整数据到串口
            if self.serial_comm.is_connected:
                # 获取最新的第1帧、第2帧和第3帧数据
                latest_frame1_data = self.get_latest_frame1_data()
                latest_frame2_data = self.get_latest_frame2_data()
                latest_frame3_data = self.get_latest_frame3_data()
                
                if latest_frame1_data and latest_frame2_data and latest_frame3_data:
                    self.serial_comm.send_motor_data(
                        displacement_cmd=latest_frame1_data.get('displacement_cmd', 0.0),
                        displacement_feedback=latest_frame1_data.get('displacement_feedback', 0.0),
                        motor_speed=parsed.get('motor_speed', 0),  # 从第5帧获取电机转速
                        motor_id_current=latest_frame2_data.get('motor_id_current', 0.0),
                        dc_voltage=latest_frame2_data.get('dc_voltage', 0.0),
                        motor_iq_current=latest_frame2_data.get('motor_iq_current', 0.0),
                        motor_ia_current=latest_frame3_data.get('motor_ia_current', 0.0),  # 从第3帧获取
                        motor_ib_current=latest_frame3_data.get('motor_ib_current', 0.0)   # 从第3帧获取
                    )
    
    def connect_serial(self):
        """连接串口"""
        try:
            if self.serial_comm.connect():
                self.serial_connect_btn.setEnabled(False)
                self.serial_disconnect_btn.setEnabled(True)
                self.serial_status_label.setText("串口状态: 已连接")
                self.serial_status_label.setStyleSheet("QLabel { color: green; }")
                self.statusBar().showMessage("串口连接成功")
            else:
                QMessageBox.warning(self, "连接失败", "无法连接到串口11，请检查串口是否可用")
        except Exception as e:
            QMessageBox.critical(self, "连接错误", f"串口连接失败: {str(e)}")
    
    def disconnect_serial(self):
        """断开串口连接"""
        try:
            self.serial_comm.disconnect()
            self.serial_connect_btn.setEnabled(True)
            self.serial_disconnect_btn.setEnabled(False)
            self.serial_status_label.setText("串口状态: 未连接")
            self.serial_status_label.setStyleSheet("QLabel { color: red; }")
            self.statusBar().showMessage("串口已断开")
        except Exception as e:
            QMessageBox.critical(self, "断开错误", f"串口断开失败: {str(e)}")
                    
    def toggle_legend(self):
        """切换图例显示状态"""
        try:
            if self.legend_btn.isChecked():
                self.legend.show()
                self.legend_btn.setText("隐藏图例")
            else:
                self.legend.hide()
                self.legend_btn.setText("显示图例")
        except Exception as e:
            pass
    
    def toggle_auto_scale(self):
        """切换自动缩放状态"""
        try:
            if self.auto_scale_btn.isChecked():
                self.plot_widget.enableAutoRange(axis='y')
                self.auto_scale_btn.setText("自动缩放")
            else:
                self.plot_widget.disableAutoRange(axis='y')
                self.auto_scale_btn.setText("固定缩放")
        except Exception as e:
            pass
        

    def send_command_frame(self):
        """
        发送位移/速度指令帧
        根据新协议要求：
        - 位移指令：CAN ID 0x01121101，Byte0为0xFF表示有效，Byte1~Byte4为位移值（4字节，0.01mm量纲）
        - 扩展帧格式，大端序传输
        """
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
            
            # 位移指令：输入mm值，转换为0.01mm单位，使用4个字节存储
            # 根据协议：Byte1~Byte4为位移指令，无符号整型，0.01mm量纲
            # 例如：输入10.5mm，发送1050 (10.5 * 100)
            raw_value = int(value * 100)  # 转换为0.01mm单位
            data[1:5] = raw_value.to_bytes(4, byteorder='big', signed=False)  # 4字节，大端序，无符号
            
            # 显示发送信息
            data_hex = ' '.join([f'{b:02X}' for b in data])
            info_msg = f"发送位移指令: {value} mm (原始值: {raw_value}, 状态: {enable_status}, 数据: {data_hex})"
            
            # 调试打印：16进制指令
            print(f"[DEBUG] 位移指令 16进制: {data_hex}")
            print(f"[DEBUG] CAN ID: 0x{can_id:08X}")
            print(f"[DEBUG] 位移值: {value} mm -> {raw_value} (0.01mm单位)")
            
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
        
        
        # 发送指令
        data_str = ' '.join([f'{b:02X}' for b in data])
        
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
        
        
        # 发送
        ret = self.zcanlib.Transmit(self.chn_handle, msg, 1)
        if ret == 1:
            info_msg = f"电磁阀指令发送成功 - 电磁阀0:{solenoid0_state}, 电磁阀1:{solenoid1_state}, 电磁阀2:{solenoid2_state}"
            self.statusBar().showMessage(info_msg, 3000)  # 显示3秒
        else:
            QMessageBox.warning(self, "失败", f"电磁阀指令发送失败，返回值: {ret}")
            
    def append_data_point(self, parsed_dict):
        """添加数据点到绘图数据"""
        try:
            # 确保绘图数据已初始化
            if not hasattr(self, 'plot_data') or not self.plot_data:
                self.init_plot_data()
                
            # 记录时间 - 使用相对时间（从程序启动开始计算）
            if not hasattr(self, '_start_time'):
                self._start_time = time.time()
            current_time = time.time() - self._start_time
            
            # 时间间隔采样控制（0.1秒间隔，提高采样频率以显示平滑的正弦波）
            time_interval = 0.1  # 0.1秒间隔，提高采样频率
            
            # 检查是否需要添加新的数据点
            should_add_point = False
            if len(self.time_data) == 0:
                # 第一个数据点，直接添加
                should_add_point = True
            else:
                # 检查是否达到了时间间隔
                time_since_last = current_time - self.time_data[-1]
                if time_since_last >= time_interval:
                    should_add_point = True
            
            if should_add_point:
                # 确保时间数据是递增的
                if len(self.time_data) > 0 and current_time <= self.time_data[-1]:
                    current_time = self.time_data[-1] + time_interval
                    
                self.time_data.append(current_time)
            

            
            # 添加各个参数的数据
            if should_add_point:
                for param_name in self.plot_data:
                    if param_name in parsed_dict:
                        value = parsed_dict[param_name]
                        # 验证数值的合理性
                        if isinstance(value, (int, float)) and not np.isnan(value) and not np.isinf(value):
                            self.plot_data[param_name].append(value)
                        else:
                            # 如果数值无效，使用上一个值或NaN
                            if self.plot_data[param_name]:
                                self.plot_data[param_name].append(self.plot_data[param_name][-1])
                            else:
                                self.plot_data[param_name].append(np.nan)
                    else:
                        # 如果没有新数据，保持上一个值或使用NaN
                        if self.plot_data[param_name]:
                            self.plot_data[param_name].append(self.plot_data[param_name][-1])
                        else:
                            self.plot_data[param_name].append(np.nan)
                
                # 如果启用了数据记录，同时记录到记录数组中
                if self.data_recording_enabled:
                    # 记录时间数据（相对于记录开始时间）
                    record_time = current_time - self.recording_start_time
                    self.recorded_time_data.append(record_time)
                    
                    # 记录各参数数据
                    for param_name in self.plot_data:
                        if param_name not in self.recorded_plot_data:
                            self.recorded_plot_data[param_name] = []
                        
                        if param_name in parsed_dict:
                            value = parsed_dict[param_name]
                            if isinstance(value, (int, float)) and not np.isnan(value) and not np.isinf(value):
                                self.recorded_plot_data[param_name].append(value)
                            else:
                                # 如果数值无效，使用上一个值或NaN
                                if self.recorded_plot_data[param_name]:
                                    self.recorded_plot_data[param_name].append(self.recorded_plot_data[param_name][-1])
                                else:
                                    self.recorded_plot_data[param_name].append(np.nan)
                        else:
                            # 如果没有新数据，保持上一个值或使用NaN
                            if self.recorded_plot_data[param_name]:
                                self.recorded_plot_data[param_name].append(self.recorded_plot_data[param_name][-1])
                            else:
                                self.recorded_plot_data[param_name].append(np.nan)
            
            # 简单的滑动窗口数据管理：保持固定大小的数据窗口
            window_size = 3600  # 保持1小时的数据（3600个数据点，10Hz采样）
            
            # 当数据量超过窗口大小时，删除最旧的数据
            if len(self.time_data) > window_size:
                # 删除最旧的数据，保持窗口大小
                self.time_data = self.time_data[-window_size:]
                for param_name in self.plot_data:
                    if len(self.plot_data[param_name]) > window_size:
                        self.plot_data[param_name] = self.plot_data[param_name][-window_size:]
                        

            
            # 更新状态栏显示最新数据时间
            if should_add_point and len(self.time_data) % 50 == 0:  # 每50个数据点更新一次状态，减少输出频率
                # 计算数据接收频率
                if len(self.time_data) > 1:
                    time_span = self.time_data[-1] - self.time_data[0]
                    if time_span > 0:
                        data_rate = len(self.time_data) / time_span
                        self.statusBar().showMessage(f"数据接收中 - 时间: {current_time:.1f}s, 数据点: {len(self.time_data)}, 频率: {data_rate:.1f}Hz, 显示范围: 15分钟, 采样间隔: 0.1秒")
                    else:
                        self.statusBar().showMessage(f"数据接收中 - 时间: {current_time:.1f}s, 数据点: {len(self.time_data)}, 显示范围: 15分钟, 采样间隔: 0.1秒")
                else:
                    self.statusBar().showMessage(f"数据接收中 - 时间: {current_time:.1f}s, 数据点: {len(self.time_data)}, 显示范围: 15分钟, 采样间隔: 0.1秒")
                        
        except Exception as e:
            # 记录错误但不中断程序
            pass
    def start_sine_command(self):
        """开始正弦指令"""
        if not self.chn_handle:
            QMessageBox.warning(self, "警告", "请先打开设备!")
            return
            
        # 获取正弦指令参数
        center = self.sine_center_spin.value()
        amplitude = self.sine_amplitude_spin.value()
        frequency = self.sine_frequency_spin.value()
        enable = self.sine_enable_checkbox.isChecked()
        
        if not enable:
            QMessageBox.warning(self, "警告", "请先使能正弦指令!")
            return
            
        # 记录开始时间
        self.sine_start_time = time.time()
        
        # 创建定时器，发送频率为200Hz (5ms间隔，符合协议要求)
        self.sine_timer = QTimer()
        self.sine_timer.timeout.connect(lambda: self.send_sine_frame(center, amplitude, frequency))
        self.sine_timer.start(5)  # 5ms = 200Hz，符合协议传输周期要求
        
        # 更新UI状态
        self.start_sine_btn.setEnabled(False)
        self.stop_sine_btn.setEnabled(True)
        self.sine_center_spin.setEnabled(False)
        self.sine_amplitude_spin.setEnabled(False)
        self.sine_frequency_spin.setEnabled(False)
        self.sine_enable_checkbox.setEnabled(False)
        
        self.statusBar().showMessage(f"开始正弦指令 - 中心:{center}mm, 振幅:{amplitude}mm, 频率:{frequency}Hz (传输周期:5ms, 符合协议要求)")
        
    def stop_sine_command(self):
        """停止正弦指令"""
        if self.sine_timer:
            self.sine_timer.stop()
            self.sine_timer = None
            
        # 更新UI状态
        self.start_sine_btn.setEnabled(True)
        self.stop_sine_btn.setEnabled(False)
        self.sine_center_spin.setEnabled(True)
        self.sine_amplitude_spin.setEnabled(True)
        self.sine_frequency_spin.setEnabled(True)
        self.sine_enable_checkbox.setEnabled(True)
        
        self.statusBar().showMessage("正弦指令已停止")
        
    def send_sine_frame(self, center, amplitude, frequency):
        """
        发送正弦位移指令
        根据新协议要求：
        - CAN ID: 0x01121101
        - Byte0: 0xFF表示指令有效
        - Byte1~Byte4: 位移指令，4字节，无符号整型，0.01mm量纲
        - 传输周期：5ms (200Hz)
        """
        if not self.chn_handle or not self.sine_start_time:
            return
            
        # 计算当前时间
        current_time = time.time()
        elapsed_time = current_time - self.sine_start_time
        
        # 计算正弦值
        # 正弦波公式: y = center + amplitude * sin(2π * frequency * t)
        sine_value = center + amplitude * np.sin(2 * np.pi * frequency * elapsed_time)
        
        # 限制在有效范围内
        sine_value = max(0, min(1500.0, sine_value))  # 限制在0-1500mm范围内
        
        # 构建CAN帧
        can_id = 0x01121101
        data = bytearray(8)
        
        # 根据使能状态设置Byte0
        if self.sine_enable_checkbox.isChecked():
            data[0] = 0xFF  # 指令有效，位移使能
        else:
            data[0] = 0x00  # 指令无效，位移非使能
            
        # 位移指令：转换为0.01mm单位，使用4个字节存储
        # 根据协议：Byte1~Byte4为位移指令，无符号整型，0.01mm量纲
        raw_value = int(sine_value * 100)  # 转换为0.01mm单位
        data[1:5] = raw_value.to_bytes(4, byteorder='big', signed=False)  # 4字节，大端序，无符号
        
        # 组帧
        frame = zlgcan.ZCAN_CAN_FRAME()
        frame.can_id = can_id
        frame.eff = 1 if can_id > 0x7FF else 0  # 扩展帧
        frame.rtr = 0  # 数据帧
        frame.can_dlc = len(data)
        
        for i in range(len(data)):
            frame.data[i] = data[i]
            
        msg = zlgcan.ZCAN_Transmit_Data()
        msg.frame = frame
        msg.transmit_type = 0  # 正常发送
        
        # 发送
        ret = self.zcanlib.Transmit(self.chn_handle, msg, 1)
        if ret != 1:
            pass  # 静默处理发送失败
            
    def toggle_data_recording(self):
        """切换数据记录状态"""
        if not self.data_recording_enabled:
            # 开始记录
            self.data_recording_enabled = True
            self.recording_start_time = time.time() - self._start_time if hasattr(self, '_start_time') else 0
            self.recorded_time_data = []
            self.recorded_plot_data = {}
            
            # 初始化记录数据数组
            param_names = [
                'displacement_cmd', 'displacement_feedback', 'motor_speed', 'pressure5',
                'dc_voltage', 'motor_id_current', 'motor_iq_current', 'igbt_temp',
                'motor_ia_current', 'motor_ib_current', 'pt100_a', 'pt100_b', 'pt100_c',
                'pressure1', 'pressure2', 'pressure3', 'pressure4'
            ]
            for param_name in param_names:
                self.recorded_plot_data[param_name] = []
            
            # 暂停自动保存功能
            if self.auto_save_enabled:
                self.auto_save_btn.setChecked(False)
                self.auto_save_enabled = False
                self.stop_auto_save()
                self.statusBar().showMessage("数据记录已开始 - 自动保存已暂停")
            else:
                self.statusBar().showMessage("数据记录已开始")
            
            # 更新按钮文字
            self.save_btn.setText("停止记录")
        else:
            # 停止记录并保存
            self.data_recording_enabled = False
            self.save_recorded_data()
            
            # 更新按钮文字
            self.save_btn.setText("开始记录")
            self.statusBar().showMessage("数据记录已停止")
    
    def save_recorded_data(self):
        """保存记录的数据"""
        try:
            # 检查是否有记录的数据
            if not self.recorded_time_data or len(self.recorded_time_data) == 0:
                QMessageBox.warning(self, "警告", "没有记录的数据可保存!")
                return
            
            # 创建result文件夹
            result_dir = "result"
            if not os.path.exists(result_dir):
                os.makedirs(result_dir)
            
            # 生成文件名（包含时间戳）
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"manual_record_{timestamp}.csv"
            filepath = os.path.join(result_dir, filename)
            
            # 准备保存的数据
            save_data = []
            
            # 添加时间列
            time_column = ["时间(s)"] + [f"{t:.3f}" for t in self.recorded_time_data]
            save_data.append(time_column)
            
            # 添加各参数的数据列
            param_names_cn = {
                "displacement_cmd": "位移指令(mm)",
                "displacement_feedback": "位移反馈(mm)",
                "motor_speed": "电机转速(r/min)",
                "pressure5": "压力5(Mpa)",
                "dc_voltage": "DC母线电压(V)",
                "motor_id_current": "电机Id电流(A)",
                "motor_iq_current": "电机Iq电流(A)",
                "igbt_temp": "IGBT温度(°C)",
                "motor_ia_current": "电机Ia电流(A)",
                "motor_ib_current": "电机Ib电流(A)",
                "pt100_a": "PT100-A(°C)",
                "pt100_b": "PT100-B(°C)",
                "pt100_c": "PT100-C(°C)",
                "pressure1": "压力1(Mpa)",
                "pressure2": "压力2(Mpa)",
                "pressure3": "压力3(Mpa)",
                "pressure4": "压力4(Mpa)"
            }
            
            for param_name, param_cn in param_names_cn.items():
                if param_name in self.recorded_plot_data and len(self.recorded_plot_data[param_name]) > 0:
                    # 确保数据长度与时间数据一致
                    data_length = min(len(self.recorded_time_data), len(self.recorded_plot_data[param_name]))
                    param_data = [param_cn] + [f"{self.recorded_plot_data[param_name][i]:.3f}" if i < len(self.recorded_plot_data[param_name]) else "" for i in range(data_length)]
                    save_data.append(param_data)
            
            # 保存为CSV文件
            with open(filepath, 'w', newline='', encoding='utf-8-sig') as f:
                import csv
                writer = csv.writer(f)
                # 转置数据以正确的格式写入
                max_length = max(len(row) for row in save_data)
                for i in range(max_length):
                    row = []
                    for col in save_data:
                        if i < len(col):
                            row.append(col[i])
                        else:
                            row.append("")
                    writer.writerow(row)
            
            # 显示成功消息
            QMessageBox.information(self, "成功", f"记录数据已保存到: {filename}\n记录时长: {self.recorded_time_data[-1]:.1f}秒\n数据点数: {len(self.recorded_time_data)}")
            
            # 清空记录数据
            self.recorded_time_data = []
            self.recorded_plot_data = {}
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存记录数据失败: {str(e)}")
    
    def toggle_auto_save(self):
        """切换自动保存状态"""
        if self.auto_save_btn.isChecked():
            self.auto_save_enabled = True
            self.start_auto_save()
            self.statusBar().showMessage("自动保存已启动 - 每10分钟保存一次数据")
        else:
            self.auto_save_enabled = False
            self.stop_auto_save()
            self.statusBar().showMessage("自动保存已停止")
    
    def start_auto_save(self):
        """启动自动保存"""
        if not self.auto_save_timer:
            self.auto_save_timer = QTimer()
            self.auto_save_timer.timeout.connect(self.auto_save_data)
            self.auto_save_timer.start(600000)  # 10分钟 = 600000毫秒
            self.last_save_time = time.time()
    
    def stop_auto_save(self):
        """停止自动保存"""
        if self.auto_save_timer:
            self.auto_save_timer.stop()
            self.auto_save_timer = None
    
    def auto_save_data(self):
        """自动保存数据"""
        try:
            # 检查是否有数据需要保存
            if not self.time_data or len(self.time_data) == 0:
                return
            
            # 创建result文件夹
            result_dir = "result"
            if not os.path.exists(result_dir):
                os.makedirs(result_dir)
            
            # 生成文件名（包含时间戳）
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.save_counter += 1
            filename = f"auto_save_{timestamp}_{self.save_counter:03d}.csv"
            filepath = os.path.join(result_dir, filename)
            
            # 准备保存的数据
            save_data = []
            
            # 添加时间列
            time_column = ["时间(s)"] + [f"{t:.3f}" for t in self.time_data]
            save_data.append(time_column)
            
            # 添加各参数的数据列
            param_names_cn = {
                "displacement_cmd": "位移指令(mm)",
                "displacement_feedback": "位移反馈(mm)",
                "motor_speed": "电机转速(r/min)",
                "pressure5": "压力5(Mpa)",
                "dc_voltage": "DC母线电压(V)",
                "motor_id_current": "电机Id电流(A)",
                "motor_iq_current": "电机Iq电流(A)",
                "igbt_temp": "IGBT温度(°C)",
                "motor_ia_current": "电机Ia电流(A)",
                "motor_ib_current": "电机Ib电流(A)",
                "pt100_a": "PT100-A(°C)",
                "pt100_b": "PT100-B(°C)",
                "pt100_c": "PT100-C(°C)",
                "pressure1": "压力1(Mpa)",
                "pressure2": "压力2(Mpa)",
                "pressure3": "压力3(Mpa)",
                "pressure4": "压力4(Mpa)"
            }
            
            for param_name, param_cn in param_names_cn.items():
                if param_name in self.plot_data and len(self.plot_data[param_name]) > 0:
                    # 确保数据长度与时间数据一致
                    data_length = min(len(self.time_data), len(self.plot_data[param_name]))
                    param_data = [param_cn] + [f"{self.plot_data[param_name][i]:.3f}" if i < len(self.plot_data[param_name]) else "" for i in range(data_length)]
                    save_data.append(param_data)
            
            # 保存为CSV文件
            with open(filepath, 'w', newline='', encoding='utf-8-sig') as f:
                import csv
                writer = csv.writer(f)
                # 转置数据以正确的格式写入
                max_length = max(len(row) for row in save_data)
                for i in range(max_length):
                    row = []
                    for col in save_data:
                        if i < len(col):
                            row.append(col[i])
                        else:
                            row.append("")
                    writer.writerow(row)
            
            # 更新状态栏
            current_time = time.strftime("%H:%M:%S")
            self.statusBar().showMessage(f"自动保存完成 - {current_time} - 文件: {filename}")
            
        except Exception as e:
            self.statusBar().showMessage(f"自动保存失败: {str(e)}")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        try:
            # 停止自动保存
            if self.auto_save_enabled:
                self.stop_auto_save()
            
            # 停止正弦指令
            if self.sine_timer:
                self.stop_sine_command()
            
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
            event.accept()
           
def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 使用Fusion样式，看起来更现代
    
    window = CANMonitorEnhancedGUI()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 