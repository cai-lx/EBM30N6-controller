import serial
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import logging
from datetime import datetime

class EBM30N6_FEG_Controller:
    
    def __init__(self):
        self.ser = None
        self.monitoring = False
        self.monitor_thread = None
        self._command_lock = threading.Lock()
        self._connection_lock = threading.Lock()
        
        # 加热电流限制阈值 - 默认值
        self.HEATER_CURRENT_LIMIT = 100
        
        # 默认端口设为COM3
        self.DEFAULT_PORT = "COM3"
        
        # 设备状态变量
        self.beam_voltage = 0.0
        self.beam_current = 0.0
        self.heater_voltage = 0.0
        self.heater_current = 0.0
        self.suppressor_voltage = 0.0
        self.suppressor_current = 0.0
        self.extractor_voltage = 0.0
        self.extractor_current = 0.0
        self.extractor_trip_current = 735.0
        
        # 输出状态
        self.beam_enabled = False
        self.heater_enabled = False
        self.suppressor_enabled = False
        self.extractor_enabled = False
        
        # 系统状态
        self.system_status = {
            "Power On Reset": False, "Vacuum Interlock": False,
            "Input Voltage out of range": False, "Beam Over Voltage Trip": False,
            "Beam Over Current Trip": False, "Extractor Over Current Trip": False,
            "Heater Open Circuit": False, "Heater Current Trip": False,
            "Over Temperature": False, "Suppressor Over Current Trip": False,
            "Arc Trip": False, "Suppressor Regulation Trip": False,
            "Heater Regulation Trip": False, "Extractor Regulation Trip": False,
            "Rx CRC Error": False, "Rx Error": False
        }
        
        self.setup_logger()
    
    def setup_logger(self):
        """设置日志系统"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('ebm30_controller_final.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
    
    def is_connected(self):
        """检查连接状态"""
        return self.ser and self.ser.is_open
    
    def get_connection(self):
        """获取连接状态信息"""
        if self.is_connected():
            return {
                "connected": True,
                "port": self.ser.port if self.ser else "Unknown",
                "baudrate": self.ser.baudrate if self.ser else "Unknown"
            }
        else:
            return {"connected": False}
    
    def reset_supply(self):
        """复位设备"""
        if not self.is_connected():
            self.logger.warning("设备未连接，无法复位")
            return False
        
        try:
            # 关闭所有输出
            self.switch_supply("Beam", False)
            self.switch_supply("Heater", False)
            self.switch_supply("Suppressor", False)
            self.switch_supply("Extractor", False)
            
            time.sleep(0.5)
            
            # 重置参数到默认值
            self.set_beam_voltage(0)
            self.set_heater_current(0)
            self.set_suppressor_voltage(0)
            self.set_extractor_voltage(0)
            
            self.logger.info("设备复位完成")
            return True
            
        except Exception as e:
            self.logger.error(f"复位设备时出错: {e}")
            return False
    
    def connect(self, port=None, baudrate=115200):
        """连接到设备"""
        # 如果没有指定端口，使用默认端口
        if port is None:
            port = self.DEFAULT_PORT
            
        with self._connection_lock:
            try:
                if self.is_connected():
                    self.logger.warning("设备已连接，先断开")
                    self.disconnect()
                
                self.logger.info(f"正在连接到 {port}...")
                self.ser = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=2,
                    write_timeout=2
                )
                
                # 清空缓冲区
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
                # 发送初始化序列 - 修复关键：按照官方GUI的顺序
                time.sleep(0.5)
                init_commands = [
                    ("01", "7F"),  # Get Connection
                    ("0199", "0"),  # Set working mode
                    ("050", ""),    # Get firmware version - main
                    ("051", ""),    # Get firmware version - floating deck
                    ("080", ""),    # Get beam voltage target
                    ("081", "")     # Get beam voltage monitor
                ]
                
                for cmd, arg in init_commands:
                    result = self.send_command(cmd, arg)
                    if result:
                        self.logger.debug(f"初始化命令 {cmd} 成功")
                    else:
                        self.logger.warning(f"初始化命令 {cmd} 无响应")
                    time.sleep(0.1)
                
                # 测试连接
                test_result = self.send_command("02")
                if test_result:
                    self.logger.info(f"成功连接到设备: {port}")
                    return True
                else:
                    self.logger.error("连接测试失败")
                    self.disconnect()
                    return False
                    
            except Exception as e:
                self.logger.error(f"连接失败: {e}")
                if self.ser:
                    try:
                        self.ser.close()
                    except:
                        pass
                return False
    
    def disconnect(self):
        """断开设备连接"""
        with self._connection_lock:
            self.logger.info("开始断开连接...")
            
            # 先停止监控
            self.stop_monitoring()
            
            # 等待监控线程完全停止
            time.sleep(0.5)
            
            # 关闭串口
            if self.ser:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                        self.logger.info("串口已关闭")
                except Exception as e:
                    self.logger.error(f"关闭串口时出错: {e}")
                finally:
                    self.ser = None
            
            self.logger.info("设备已断开连接")
    
    def calculate_checksum(self, message):
        """计算校验和 - 修复版本，基于诊断结果"""
        # 根据诊断结果，使用简单的算法
        loop_total = sum(ord(char) for char in message)
        
        # 使用诊断中有效的参数
        HV_CHECKSUM_NEGATION = 0x1E0
        HV_LOWER_CHECKSUM_BOUND = 0x40
        HV_UPPER_CHECKSUM_BOUND = 0x7F
        
        checksum = HV_CHECKSUM_NEGATION - loop_total
        checksum &= HV_UPPER_CHECKSUM_BOUND
        checksum |= HV_LOWER_CHECKSUM_BOUND
        
        if checksum < HV_LOWER_CHECKSUM_BOUND or checksum > HV_UPPER_CHECKSUM_BOUND:
            # 如果超出范围，使用备选算法
            checksum = (0x100 - (loop_total & 0xFF)) & 0x7F
            checksum |= HV_LOWER_CHECKSUM_BOUND
        
        return f"{checksum:02X}"
    
    def send_command(self, cmd, args=None):
        """发送命令到设备 - 修复版本"""
        if not self.is_connected():
            self.logger.warning("尝试发送命令但设备未连接")
            return None
        
        with self._command_lock:
            try:
                # 构建命令 - 关键修复：根据诊断结果调整格式
                # 对于没有参数的命令，直接发送命令
                if args is None or args == "":
                    base_cmd = f":{cmd}"
                else:
                    # 有参数时，参数直接跟在命令后面，没有空格
                    base_cmd = f":{cmd}{args}"
                
                checksum = self.calculate_checksum(base_cmd[1:])  # 去掉冒号计算校验和
                
                # 构建完整命令：基础命令 + 空格 + 校验和 + 换行
                full_cmd = f"{base_cmd} {checksum}\n"
                
                self.logger.debug(f"发送命令: {full_cmd.strip()}")
                
                # 双重检查连接状态
                if not self.is_connected():
                    self.logger.error("发送命令前连接已断开")
                    return None
                
                # 发送命令
                self.ser.write(full_cmd.encode())
                time.sleep(0.05)
                
                # 读取响应
                response = self.ser.readline().decode().strip()
                
                if response:
                    self.logger.debug(f"接收响应: {response}")
                    return self.parse_response(response)
                else:
                    self.logger.warning("命令无响应")
                    return None
                    
            except Exception as e:
                self.logger.error(f"通信错误: {e}")
                return None
    
    def parse_response(self, response):
        """解析设备响应"""
        if not response or not response.startswith(':'):
            return None
        
        try:
            # 移除开头的冒号
            response_content = response[1:]
            
            # 分离校验和（通常是最后2个字符）
            if len(response_content) > 2:
                # 查找最后一个空格来分离校验和
                last_space = response_content.rfind(' ')
                if last_space != -1:
                    data_part = response_content[:last_space].strip()
                    checksum = response_content[last_space+1:].strip()
                else:
                    # 如果没有空格，假设最后2个字符是校验和
                    checksum = response_content[-2:]
                    data_part = response_content[:-2].strip()
            else:
                checksum = None
                data_part = response_content
            
            # 分离命令和数据
            parts = data_part.split()
            if not parts:
                return None
            
            cmd = parts[0]
            data = parts[1:] if len(parts) > 1 else []
            
            return {
                "command": cmd, 
                "data": data, 
                "checksum": checksum,
                "raw": response
            }
            
        except Exception as e:
            self.logger.error(f"解析响应错误: {e}")
            return None

    # 其余的方法保持不变...
    def get_short_status(self):
        """02 - 获取状态"""
        result = self.send_command("02")
        if result:
            # 从响应中提取状态信息
            status_data = result["command"][2:] if len(result["command"]) > 2 else "00000000"
            if result["data"]:
                status_data = result["data"][0]
            
            self.update_system_status(status_data)
            self.update_output_states(status_data)
        return result
    
    def update_output_states(self, status_hex):
        """从状态寄存器更新输出状态"""
        try:
            if len(status_hex) >= 8:
                status_int = int(status_hex[:8], 16)
                # 字节1的位0-3分别对应各输出的ON/OFF状态
                self.beam_enabled = bool(status_int & (1 << 0))
                self.extractor_enabled = bool(status_int & (1 << 1))
                self.suppressor_enabled = bool(status_int & (1 << 2))
                self.heater_enabled = bool(status_int & (1 << 3))
        except Exception as e:
            self.logger.error(f"更新输出状态错误: {e}")
    
    def switch_supply(self, supply, state):
        """03 - 控制输出开关"""
        cmd_map = {
            "Beam": ("00", "01"),
            "Extractor": ("02", "03"),
            "Suppressor": ("04", "05"),
            "Heater": ("06", "07")
        }
        
        if supply in cmd_map:
            cmd = cmd_map[supply][0] if state else cmd_map[supply][1]
            result = self.send_command("03", cmd)
            
            if result:
                time.sleep(0.2)
                # 更新状态
                self.get_short_status()
                return True
            return False
        return False
    
    def set_beam_voltage(self, voltage):
        """09 - 设置束电压"""
        if 0 <= voltage <= 30000:
            # 格式化为xxxxx.x格式
            voltage_str = f"{voltage:06.1f}"
            return self.send_command("09", voltage_str)
        return None
    
    def set_heater_current(self, current):
        """29 - 设置加热器电流 - 包含安全限制"""
        # 检查是否超过限制阈值
        if current > self.HEATER_CURRENT_LIMIT:
            # 超过限制时设为限制值，而不是0
            limited_current = self.HEATER_CURRENT_LIMIT
            messagebox.showwarning(
                "Safety Warning",
                f"Warning: Heater current setting exceeds maximum limit!\n"
                f"Requested value: {current} mA\n"
                f"Automatically set to maximum allowed: {limited_current} mA"
            )
            # 递归调用设置限制值
            return self.set_heater_current(limited_current)
        
        if 0 <= current <= 3000:
            # 格式化为xxxx.x格式
            current_str = f"{current:05.1f}"
            return self.send_command("29", current_str)
        return None
    
    def set_suppressor_voltage(self, voltage):
        """1F - 设置抑制器电压"""
        if 0 <= voltage <= 1000:
            voltage_str = f"{voltage:06.1f}"
            return self.send_command("1F", voltage_str)
        return None
    
    def set_extractor_voltage(self, voltage):
        """15 - 设置提取器电压"""
        if 0 <= voltage <= 10000:
            voltage_str = f"{voltage:06.1f}"
            return self.send_command("15", voltage_str)
        return None
    
    def set_extractor_trip_current(self, current):
        """1D - 设置提取器跳闸电流"""
        if 50 <= current <= 770:
            return self.send_command("1D", f"{int(current)}")
        return None

    # ===== 监测数据读取函数 =====
    
    def get_beam_voltage_monitor(self):
        """08 - 获取束电压监测值"""
        result = self.send_command("08", "1")
        if result:
            try:
                # 根据诊断结果调整解析逻辑
                if result["data"]:
                    voltage_str = result["data"][0]
                else:
                    # 从command字段提取
                    voltage_str = result["command"][2:]
                self.beam_voltage = float(voltage_str)
                return self.beam_voltage
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析束电压错误: {e}")
        return None
    
    def get_beam_current_monitor(self):
        """0E - 获取束电流监测值"""
        result = self.send_command("0E")
        if result:
            try:
                # 束电流数据在command字段中
                current_str = result["command"][2:]
                self.beam_current = float(current_str)
                return self.beam_current
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析束电流错误: {e}")
        return None
    
    def get_heater_voltage_monitor(self):
        """26 - 获取加热器电压监测值"""
        result = self.send_command("26")
        if result:
            try:
                if result["data"]:
                    voltage_str = result["data"][0]
                else:
                    voltage_str = result["command"][2:]
                # 修复：设备返回的是mV，需要转换为V
                self.heater_voltage = float(voltage_str) / 1000.0
                return self.heater_voltage
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析加热器电压错误: {e}")
        return None
    
    def get_heater_current_monitor(self):
        """28 - 获取加热器电流监测值"""
        result = self.send_command("28", "1")
        if result:
            try:
                if result["data"]:
                    current_str = result["data"][0]
                else:
                    current_str = result["command"][2:]
                self.heater_current = float(current_str)
                return self.heater_current
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析加热器电流错误: {e}")
        return None
    
    def get_suppressor_voltage_monitor(self):
        """1E - 获取抑制器电压监测值"""
        result = self.send_command("1E", "1")
        if result:
            try:
                if result["data"]:
                    voltage_str = result["data"][0]
                else:
                    voltage_str = result["command"][2:]
                self.suppressor_voltage = float(voltage_str)
                return self.suppressor_voltage
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析抑制器电压错误: {e}")
        return None
    
    def get_suppressor_current_monitor(self):
        """24 - 获取抑制器电流监测值"""
        result = self.send_command("24")
        if result:
            try:
                if result["data"]:
                    current_str = result["data"][0]
                else:
                    current_str = result["command"][2:]
                self.suppressor_current = float(current_str)
                return self.suppressor_current
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析抑制器电流错误: {e}")
        return None
    
    def get_extractor_voltage_monitor(self):
        """14 - 获取提取器电压监测值"""
        result = self.send_command("14", "1")
        if result:
            try:
                if result["data"]:
                    voltage_str = result["data"][0]
                else:
                    voltage_str = result["command"][2:]
                self.extractor_voltage = float(voltage_str)
                return self.extractor_voltage
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析提取器电压错误: {e}")
        return None
    
    def get_extractor_current_monitor(self):
        """1A - 获取提取器电流监测值"""
        result = self.send_command("1A")
        if result:
            try:
                if result["data"]:
                    current_str = result["data"][0]
                else:
                    current_str = result["command"][2:]
                self.extractor_current = float(current_str)
                return self.extractor_current
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析提取器电流错误: {e}")
        return None
    
    def get_extractor_trip_current_monitor(self):
        """1C - 获取提取器跳闸电流设置值"""
        result = self.send_command("1C")
        if result:
            try:
                if result["data"]:
                    current_str = result["data"][0]
                else:
                    current_str = result["command"][2:]
                self.extractor_trip_current = float(current_str)
                return self.extractor_trip_current
            except (ValueError, IndexError) as e:
                self.logger.error(f"解析提取器跳闸电流错误: {e}")
        return None
    
    def update_all_monitors(self):
        """更新所有监测值"""
        if not self.is_connected():
            return
        
        try:
            self.get_beam_voltage_monitor()
            self.get_beam_current_monitor()
            self.get_heater_current_monitor()
            self.get_heater_voltage_monitor()
            self.get_extractor_voltage_monitor()
            self.get_extractor_current_monitor()
            self.get_suppressor_voltage_monitor()
            self.get_suppressor_current_monitor()
            self.get_extractor_trip_current_monitor()
            self.get_short_status()
        except Exception as e:
            self.logger.error(f"更新监测值时出错: {e}")
    
    def update_system_status(self, status_hex):
        """更新系统状态标志"""
        try:
            if len(status_hex) >= 8:
                status_int = int(status_hex[:8], 16)
                self.system_status["Power On Reset"] = bool(status_int & (1 << 8))
                self.system_status["Vacuum Interlock"] = bool(status_int & (1 << 10))
                self.system_status["Input Voltage out of range"] = bool(status_int & (1 << 12))
                self.system_status["Beam Over Voltage Trip"] = bool(status_int & (1 << 13))
                self.system_status["Beam Over Current Trip"] = bool(status_int & (1 << 14))
                self.system_status["Extractor Over Current Trip"] = bool(status_int & (1 << 15))
                self.system_status["Heater Open Circuit"] = bool(status_int & (1 << 16))
                self.system_status["Heater Current Trip"] = bool(status_int & (1 << 17))
                self.system_status["Over Temperature"] = bool(status_int & (1 << 18))
                self.system_status["Suppressor Over Current Trip"] = bool(status_int & (1 << 19))
                self.system_status["Arc Trip"] = bool(status_int & (1 << 20))
                self.system_status["Suppressor Regulation Trip"] = bool(status_int & (1 << 21))
                self.system_status["Heater Regulation Trip"] = bool(status_int & (1 << 22))
                self.system_status["Extractor Regulation Trip"] = bool(status_int & (1 << 23))
                self.system_status["Rx CRC Error"] = bool(status_int & (1 << 30))
                self.system_status["Rx Error"] = bool(status_int & (1 << 31))
        except Exception as e:
            self.logger.error(f"更新系统状态错误: {e}")
    
    def start_monitoring(self, interval=2000):
        """开始实时监测"""
        if self.monitoring:
            self.logger.warning("监控已在运行")
            return
            
        self.monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            args=(interval/1000,),
            daemon=True
        )
        self.monitor_thread.start()
        self.logger.info(f"开始实时监测，间隔: {interval}ms")
    
    def stop_monitoring(self):
        """停止实时监测"""
        if not self.monitoring:
            return
            
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=3)
            if self.monitor_thread.is_alive():
                self.logger.warning("监控线程未能及时停止")
            else:
                self.logger.info("监控线程已停止")
        self.monitor_thread = None
    
    def _monitoring_loop(self, interval):
        """监测循环"""
        cycle_count = 0
        while self.monitoring and self.is_connected():
            try:
                cycle_count += 1
                self.logger.debug(f"监控周期 #{cycle_count}")
                self.update_all_monitors()
                time.sleep(interval)
            except Exception as e:
                self.logger.error(f"监控循环错误: {e}")
                time.sleep(interval)
        
        self.logger.info(f"监控循环结束，总共运行 {cycle_count} 个周期")

class EBM30_GUI:
    """EBM30N6/FEG Graphical User Interface"""
    
    def __init__(self):
        self.controller = EBM30N6_FEG_Controller()
        self.root = tk.Tk()
        self.status_frames = {}
        self.setup_gui()
    
    def setup_gui(self):
        """Setup graphical user interface"""
        self.root.title("EBM30N6/FEG High Voltage Power Supply Controller")
        self.root.geometry("500x600")
        
        # 创建主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=2)
        main_frame.rowconfigure(1, weight=1)
        
        # ===== 左侧区域：控制面板和状态指示器 =====
        left_frame = ttk.LabelFrame(main_frame, text="Control Panel", padding="5", width=200)
        left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        left_frame.grid_propagate(False)
        
        # 启用控制区域
        enable_frame = ttk.Frame(left_frame)
        enable_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(enable_frame, text="Enable Supply", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W)
        
        # 使用StringVar来显示状态
        self.beam_var = tk.StringVar(value="off")
        self.heater_var = tk.StringVar(value="off")
        self.suppressor_var = tk.StringVar(value="off")
        self.extractor_var = tk.StringVar(value="off")
        
        # 创建带状态显示的复选框
        self.beam_check = ttk.Checkbutton(enable_frame, text="Beam Energy Supply", 
                                         variable=self.beam_var, onvalue="on", offvalue="off",
                                         command=lambda: self.toggle_supply("Beam", self.beam_var.get() == "on"))
        self.beam_check.grid(row=1, column=0, sticky=tk.W)
        
        self.heater_check = ttk.Checkbutton(enable_frame, text="Heater Supply",
                                          variable=self.heater_var, onvalue="on", offvalue="off",
                                          command=lambda: self.toggle_supply("Heater", self.heater_var.get() == "on"))
        self.heater_check.grid(row=2, column=0, sticky=tk.W)
        
        self.suppressor_check = ttk.Checkbutton(enable_frame, text="Suppressor Supply",
                                              variable=self.suppressor_var, onvalue="on", offvalue="off",
                                              command=lambda: self.toggle_supply("Suppressor", self.suppressor_var.get() == "on"))
        self.suppressor_check.grid(row=3, column=0, sticky=tk.W)
        
        self.extractor_check = ttk.Checkbutton(enable_frame, text="Extractor Voltage Supply",
                                             variable=self.extractor_var, onvalue="on", offvalue="off",
                                             command=lambda: self.toggle_supply("Extractor", self.extractor_var.get() == "on"))
        self.extractor_check.grid(row=4, column=0, sticky=tk.W)
        
        # 状态指示器区域
        status_frame = ttk.Frame(left_frame)
        status_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Label(status_frame, text="System Status", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=(10, 5))
        
        # 创建状态指示器的画布和滚动条
        status_canvas = tk.Canvas(status_frame, height=200, width=180)
        status_scrollbar = ttk.Scrollbar(status_frame, orient="vertical", command=status_canvas.yview)
        self.status_scrollable_frame = ttk.Frame(status_canvas)
        
        self.status_scrollable_frame.bind(
            "<Configure>",
            lambda e: status_canvas.configure(scrollregion=status_canvas.bbox("all"))
        )
        
        status_canvas.create_window((0, 0), window=self.status_scrollable_frame, anchor="nw")
        status_canvas.configure(yscrollcommand=status_scrollbar.set)
        
        status_canvas.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        status_scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        
        # 在可滚动框架中创建状态指示器
        self.status_vars = {}
        self.status_items = [
            "Power On Reset", "Vacuum Interlock", "Input Voltage out of range",
            "Beam Over Voltage Trip", "Beam Over Current Trip", "Extractor Over Current Trip",
            "Heater Open Circuit", "Heater Current Trip", "Over Temperature",
            "Suppressor Over Current Trip", "Arc Trip", "Suppressor Regulation Trip",
            "Heater Regulation Trip", "Extractor Regulation Trip", "Rx CRC Error", "Rx Error"
        ]
        
        for item in self.status_items:
            var = tk.BooleanVar()
            self.status_vars[item] = var
            frame = ttk.Frame(self.status_scrollable_frame, width=180)
            frame.pack(fill=tk.X, pady=1)
            
            # 状态指示器
            indicator = tk.Canvas(frame, width=20, height=20)
            indicator.pack(side=tk.LEFT, padx=(0, 5))
            indicator.create_oval(2, 2, 18, 18, fill="light gray", tags="indicator")
            
            ttk.Label(frame, text=item, font=('Arial', 8)).pack(side=tk.LEFT, fill=tk.X, expand=True)
            
            # 存储引用以便排序
            self.status_frames[item] = {
                "frame": frame,
                "indicator": indicator,
                "var": var
            }
        
        # 配置状态框架的网格权重
        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(1, weight=1)
        left_frame.rowconfigure(1, weight=1)
        
        # ===== 右侧区域：设置和监测 =====
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        right_frame.columnconfigure(1, weight=1)
        right_frame.rowconfigure(1, weight=1)
        
        # 设置区域
        set_frame = ttk.LabelFrame(right_frame, text="Parameter Settings", padding="5")
        set_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        set_frame.columnconfigure(1, weight=1)
        
        # 束能电压
        ttk.Label(set_frame, text="Beam Energy Voltage (V):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.beam_voltage_var = tk.DoubleVar(value=0.0)
        beam_entry = ttk.Entry(set_frame, textvariable=self.beam_voltage_var, width=8)
        beam_entry.grid(row=0, column=1, sticky=(tk.W, tk.E), pady=2, padx=(5, 0))
        beam_entry.bind('<Return>', lambda e: self.set_beam_voltage())
        
        # 加热器电流
        ttk.Label(set_frame, text="Heater Current (mA):").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.heater_current_var = tk.DoubleVar(value=0.0)
        heater_entry = ttk.Entry(set_frame, textvariable=self.heater_current_var, width=8)
        heater_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=2, padx=(5, 0))
        heater_entry.bind('<Return>', lambda e: self.set_heater_current())
        
        # 抑制器电压
        ttk.Label(set_frame, text="Suppressor Voltage (V):").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.suppressor_voltage_var = tk.DoubleVar(value=0.0)
        suppressor_entry = ttk.Entry(set_frame, textvariable=self.suppressor_voltage_var, width=8)
        suppressor_entry.grid(row=2, column=1, sticky=(tk.W, tk.E), pady=2, padx=(5, 0))
        suppressor_entry.bind('<Return>', lambda e: self.set_suppressor_voltage())
        
        # 提取器电压
        ttk.Label(set_frame, text="Extractor Voltage (V):").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.extractor_voltage_var = tk.DoubleVar(value=0.0)
        extractor_entry = ttk.Entry(set_frame, textvariable=self.extractor_voltage_var, width=8)
        extractor_entry.grid(row=3, column=1, sticky=(tk.W, tk.E), pady=2, padx=(5, 0))
        extractor_entry.bind('<Return>', lambda e: self.set_extractor_voltage())
        
        # 提取器跳闸电流
        ttk.Label(set_frame, text="Extractor Trip Current (μA):").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.extractor_trip_var = tk.DoubleVar(value=735.0)
        extractor_trip_entry = ttk.Entry(set_frame, textvariable=self.extractor_trip_var, width=8)
        extractor_trip_entry.grid(row=4, column=1, sticky=(tk.W, tk.E), pady=2, padx=(5, 0))
        extractor_trip_entry.bind('<Return>', lambda e: self.set_extractor_trip_current())
        
        # 加热器电流限制控制
        limit_control_frame = ttk.Frame(set_frame)
        limit_control_frame.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(10, 0))
        
        # 启用编辑复选框
        self.enable_limit_edit_var = tk.IntVar(value=0)
        enable_limit_check = ttk.Checkbutton(
            limit_control_frame, 
            text="Enable Heater Current Limit Edit", 
            variable=self.enable_limit_edit_var,
            command=self.toggle_limit_edit
        )
        enable_limit_check.grid(row=0, column=0, columnspan=2, sticky=tk.W)
        
        # 加热器电流限制
        ttk.Label(limit_control_frame, text="Heater Current Limit (mA):").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.heater_limit_var = tk.DoubleVar(value=self.controller.HEATER_CURRENT_LIMIT)
        self.heater_limit_entry = ttk.Entry(limit_control_frame, textvariable=self.heater_limit_var, width=8, state="disabled")
        self.heater_limit_entry.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=2, padx=(5, 0))
        self.heater_limit_entry.bind('<Return>', lambda e: self.set_heater_current_limit())
        # 添加焦点离开事件
        self.heater_limit_entry.bind('<FocusOut>', lambda e: self.on_limit_focus_out())
        
        # 监测区域
        monitor_frame = ttk.LabelFrame(right_frame, text="Real-time Monitoring", padding="5")
        monitor_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        monitor_frame.columnconfigure(1, weight=1)
        
        # 创建监测表头
        ttk.Label(monitor_frame, text="Parameter", font=('Arial', 9, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Label(monitor_frame, text="Value", font=('Arial', 9, 'bold')).grid(row=0, column=1, sticky=tk.W, pady=2)
        
        # 监测变量 - 删除"Heater Current Limit"项
        self.monitor_vars = {}
        monitor_items = [
            ("Beam Energy Voltage", "V"),
            ("Beam Energy Current", "μA"),
            ("Heater Current", "mA"),
            ("Heater Voltage", "V"),
            ("Suppressor Voltage", "V"),
            ("Suppressor Current", "μA"),
            ("Extractor Voltage", "V"),
            ("Extractor Current", "μA"),
            ("Extractor Trip Current", "μA")
            # 删除这一行：("Heater Current Limit", "mA")
        ]
        
        for i, (name, unit) in enumerate(monitor_items):
            row = i + 1
            
            # 参数名称
            ttk.Label(monitor_frame, text=f"{name} ({unit}):").grid(row=row, column=0, sticky=tk.W, pady=2)
            
            # 监测值
            monitor_var = tk.StringVar(value="0.0")
            self.monitor_vars[f"{name}_monitor"] = monitor_var
            monitor_display = tk.Label(monitor_frame, textvariable=monitor_var, 
                                     background="#F9F9F9", relief="flat",
                                     width=10, anchor="e", font=('Arial', 9))
            monitor_display.grid(row=row, column=1, sticky=(tk.W, tk.E), pady=2, padx=(5, 5))
        
        # ===== 底部控制区域 =====
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(10, 0))
        
        # 状态栏
        self.status_var = tk.StringVar(value=f"Not connected - Default Port: {self.controller.DEFAULT_PORT}")
        status_label = ttk.Label(bottom_frame, textvariable=self.status_var, relief="sunken")
        status_label.grid(row=0, column=0, columnspan=4, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # 控制按钮 - 重新排列顺序
        control_frame = ttk.Frame(bottom_frame)
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E))
        bottom_frame.columnconfigure(0, weight=1)
        
        # 重新排列按钮顺序：Connect, Reset, Disconnect, Exit
        ttk.Button(control_frame, text="Connect", command=self.connect_device).grid(row=0, column=0, padx=(0, 5))
        ttk.Button(control_frame, text="Reset", command=self.reset_device).grid(row=0, column=1, padx=(0, 5))
        ttk.Button(control_frame, text="Disconnect", command=self.disconnect_device).grid(row=0, column=2, padx=(0, 5))
        ttk.Button(control_frame, text="Exit", command=self.quit_app).grid(row=0, column=3, padx=(0, 0))
        
        # 居中控制框架内容
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        control_frame.columnconfigure(2, weight=1)
        control_frame.columnconfigure(3, weight=1)
        
        # 开始显示更新
        self.update_display()
    
    def toggle_limit_edit(self):
        """切换加热电流限制编辑状态 - 修复版本"""
        if self.enable_limit_edit_var.get() == 1:
            self.heater_limit_entry.config(state="normal")
            self.heater_limit_entry.focus_set()  # 自动聚焦到输入框
            # 保存当前值到临时变量，防止被实时更新覆盖
            self.temp_limit_value = self.heater_limit_var.get()
        else:
            self.heater_limit_entry.config(state="disabled")
            # 如果取消编辑，恢复原来的值
            self.heater_limit_var.set(self.controller.HEATER_CURRENT_LIMIT)
    
    def on_limit_focus_out(self):
        """当焦点离开限制输入框时的处理"""
        if self.enable_limit_edit_var.get() == 1:
            # 如果仍在编辑状态，但不按回车，保持当前输入的值
            try:
                new_value = float(self.heater_limit_entry.get())
                if new_value <= 0:
                    messagebox.showerror("Error", "Heater current limit must be greater than 0")
                    self.heater_limit_var.set(self.controller.HEATER_CURRENT_LIMIT)
                else:
                    # 保持用户输入的值，直到按回车确认
                    self.heater_limit_var.set(new_value)
            except ValueError:
                # 如果输入无效，恢复原来的值
                self.heater_limit_var.set(self.controller.HEATER_CURRENT_LIMIT)
    
    def set_heater_current_limit(self):
        """设置加热电流限制 - 修复版本"""
        try:
            new_limit = self.heater_limit_var.get()
            if new_limit <= 0:
                messagebox.showerror("Error", "Heater current limit must be greater than 0")
                # 恢复原来的值
                self.heater_limit_var.set(self.controller.HEATER_CURRENT_LIMIT)
                return
            
            # 更新控制器中的限制值
            self.controller.HEATER_CURRENT_LIMIT = new_limit
            self.status_var.set(f"Heater current limit set to {new_limit} mA")
            
            # 禁用编辑状态
            self.enable_limit_edit_var.set(0)
            self.heater_limit_entry.config(state="disabled")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error setting heater current limit: {e}")
            # 恢复原来的值
            self.heater_limit_var.set(self.controller.HEATER_CURRENT_LIMIT)
    
    def reset_device(self):
        """Reset the device"""
        try:
            result = self.controller.reset_supply()
            if result:
                messagebox.showinfo("Success", "Device reset successfully")
                self.status_var.set("Device reset")
            else:
                messagebox.showerror("Error", "Failed to reset device")
                self.status_var.set("Reset failed")
        except Exception as e:
            messagebox.showerror("Error", f"Error resetting device: {e}")
            self.status_var.set("Reset error")
    
    def toggle_supply(self, supply, desired_state):
        """Toggle power supply state with error handling"""
        try:
            success = self.controller.switch_supply(supply, desired_state)
            
            if not success:
                # 启动失败，显示错误并恢复复选框状态
                messagebox.showerror("Error", f"Failed to {('enable' if desired_state else 'disable')} {supply}")
                
                # 根据设备实际状态恢复复选框
                if supply == "Beam":
                    self.beam_var.set("on" if self.controller.beam_enabled else "off")
                elif supply == "Heater":
                    self.heater_var.set("on" if self.controller.heater_enabled else "off")
                elif supply == "Suppressor":
                    self.suppressor_var.set("on" if self.controller.suppressor_enabled else "off")
                elif supply == "Extractor":
                    self.extractor_var.set("on" if self.controller.extractor_enabled else "off")
                    
                self.status_var.set(f"Failed to control {supply}")
            else:
                # 成功
                state_str = "enabled" if desired_state else "disabled"
                self.status_var.set(f"{supply} {state_str} successfully")
                
        except Exception as e:
            # 异常情况，恢复复选框到设备实际状态
            messagebox.showerror("Error", f"Error controlling {supply}: {e}")
            if supply == "Beam":
                self.beam_var.set("on" if self.controller.beam_enabled else "off")
            elif supply == "Heater":
                self.heater_var.set("on" if self.controller.heater_enabled else "off")
            elif supply == "Suppressor":
                self.suppressor_var.set("on" if self.controller.suppressor_enabled else "off")
            elif supply == "Extractor":
                self.extractor_var.set("on" if self.controller.extractor_enabled else "off")
            
            self.status_var.set(f"Error controlling {supply}")
    
    def set_beam_voltage(self):
        """Set beam voltage"""
        try:
            voltage = self.beam_voltage_var.get()
            result = self.controller.set_beam_voltage(voltage)
            if not result:
                messagebox.showerror("Error", "Failed to set beam voltage")
            else:
                self.status_var.set(f"Beam voltage set to {voltage} V")
        except Exception as e:
            messagebox.showerror("Error", f"Error setting beam voltage: {e}")
    
    def set_heater_current(self):
        """Set heater current"""
        try:
            current = self.heater_current_var.get()
            result = self.controller.set_heater_current(current)
            if result and "error" in result:
                # 错误已在控制器中处理
                pass
            elif result:
                self.status_var.set(f"Heater current set to {current} mA")
        except Exception as e:
            messagebox.showerror("Error", f"Error setting heater current: {e}")
    
    def set_suppressor_voltage(self):
        """Set suppressor voltage"""
        try:
            voltage = self.suppressor_voltage_var.get()
            result = self.controller.set_suppressor_voltage(voltage)
            if not result:
                messagebox.showerror("Error", "Failed to set suppressor voltage")
            else:
                self.status_var.set(f"Suppressor voltage set to {voltage} V")
        except Exception as e:
            messagebox.showerror("Error", f"Error setting suppressor voltage: {e}")
    
    def set_extractor_voltage(self):
        """Set extractor voltage"""
        try:
            voltage = self.extractor_voltage_var.get()
            result = self.controller.set_extractor_voltage(voltage)
            if not result:
                messagebox.showerror("Error", "Failed to set extractor voltage")
            else:
                self.status_var.set(f"Extractor voltage set to {voltage} V")
        except Exception as e:
            messagebox.showerror("Error", f"Error setting extractor voltage: {e}")
    
    def set_extractor_trip_current(self):
        """Set extractor trip current"""
        try:
            current = self.extractor_trip_var.get()
            result = self.controller.set_extractor_trip_current(current)
            if not result:
                messagebox.showerror("Error", "Failed to set extractor trip current")
            else:
                self.status_var.set(f"Extractor trip current set to {current} μA")
        except Exception as e:
            messagebox.showerror("Error", f"Error setting extractor trip current: {e}")
    
    def connect_device(self):
        """Connect to device - 使用默认端口"""
        try:
            success = self.controller.connect()
            if success:
                messagebox.showinfo("Success", f"Device connected successfully to {self.controller.DEFAULT_PORT}")
                self.status_var.set(f"Connected to {self.controller.DEFAULT_PORT}")
                self.controller.start_monitoring()
            else:
                self.status_var.set(f"Connection failed to {self.controller.DEFAULT_PORT}")
        except Exception as e:
            messagebox.showerror("Error", f"Error connecting to device: {e}")
            self.status_var.set("Connection error")
    
    def disconnect_device(self):
        """Disconnect from device"""
        self.controller.disconnect()
        messagebox.showinfo("Information", "Device disconnected")
        self.status_var.set(f"Not connected - Default Port: {self.controller.DEFAULT_PORT}")
    
    def update_display(self):
        """Update display"""
        # 更新监测值
        self.monitor_vars["Beam Energy Voltage_monitor"].set(f"{self.controller.beam_voltage:.1f}")
        self.monitor_vars["Beam Energy Current_monitor"].set(f"{self.controller.beam_current:.2f}")
        self.monitor_vars["Heater Current_monitor"].set(f"{self.controller.heater_current:.1f}")
        
        # 修复：Heater Voltage 单位从 mV 转换为 V
        self.monitor_vars["Heater Voltage_monitor"].set(f"{self.controller.heater_voltage:.2f}")
        
        self.monitor_vars["Suppressor Voltage_monitor"].set(f"{self.controller.suppressor_voltage:.1f}")
        self.monitor_vars["Suppressor Current_monitor"].set(f"{self.controller.suppressor_current:.2f}")
        self.monitor_vars["Extractor Voltage_monitor"].set(f"{self.controller.extractor_voltage:.1f}")
        self.monitor_vars["Extractor Current_monitor"].set(f"{self.controller.extractor_current:.2f}")
        self.monitor_vars["Extractor Trip Current_monitor"].set(f"{self.controller.extractor_trip_current:.1f}")
        
        # 删除这一部分：更新加热电流限制显示（因为已经从监测区域删除了）
        # 只需要更新参数设置区域中的限制值
        if self.enable_limit_edit_var.get() == 0:
            self.heater_limit_var.set(self.controller.HEATER_CURRENT_LIMIT)
        
        # 同步复选框状态到设备实际状态
        self.beam_var.set("on" if self.controller.beam_enabled else "off")
        self.heater_var.set("on" if self.controller.heater_enabled else "off")
        self.suppressor_var.set("on" if self.controller.suppressor_enabled else "off")
        self.extractor_var.set("on" if self.controller.extractor_enabled else "off")
        
        # 更新状态指示器并按错误状态排序
        error_items = []
        normal_items = []
        
        for status_name in self.status_items:
            status_value = self.controller.system_status.get(status_name, False)
            self.status_vars[status_name].set(status_value)
            
            # 更新指示器颜色
            frame_info = self.status_frames[status_name]
            if status_value:
                color = "red"  # 错误状态为红色
                error_items.append(status_name)
            else:
                color = "green"  # 正常状态为绿色
                normal_items.append(status_name)
            
            frame_info["indicator"].itemconfig("indicator", fill=color)
        
        # 重新打包状态项，错误项在前
        for status_name in error_items + normal_items:
            frame_info = self.status_frames[status_name]
            frame_info["frame"].pack_forget()  # 从当前位置移除
            frame_info["frame"].pack(fill=tk.X, pady=1)  # 按正确顺序重新添加
        
        # 500ms后再次更新
        self.root.after(500, self.update_display)
    
    def quit_app(self):
        """Exit application"""
        self.controller.stop_monitoring()
        self.controller.disconnect()
        self.root.quit()
        self.root.destroy()
    
    def run(self):
        """Run application"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.quit_app()


def main():
    """Main function"""
    print("Starting EBM30N6/FEG High Voltage Power Supply Controller...")
    
    app = EBM30_GUI()
    app.run()


if __name__ == "__main__":
    main()
