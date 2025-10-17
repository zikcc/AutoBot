#!/usr/bin/env python
# autobot_voice/src/voice_command.py
import rospy
import serial
import threading
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VoiceCommandNode:
    def __init__(self):
        rospy.init_node('voice_command', anonymous=True)
        
        # 参数配置
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.command_timeout = rospy.get_param('~command_timeout', 3.0)
        
        # 语音命令到控制指令的映射
        self.command_mapping = {
            # 中文命令
            "前进": self.create_twist(0.5, 0.0),
            "后退": self.create_twist(-0.3, 0.0),
            "左转": self.create_twist(0.0, 1.0),
            "右转": self.create_twist(0.0, -1.0),
            "停止": self.create_twist(0.0, 0.0),
            "加速": self.create_twist(0.8, 0.0),
            "减速": self.create_twist(0.3, 0.0),
            
            # 英文命令
            "forward": self.create_twist(0.5, 0.0),
            "backward": self.create_twist(-0.3, 0.0),
            "turn left": self.create_twist(0.0, 1.0),
            "turn right": self.create_twist(0.0, -1.0),
            "stop": self.create_twist(0.0, 0.0),
            "faster": self.create_twist(0.8, 0.0),
            "slower": self.create_twist(0.3, 0.0),
        }
        
        # ROS发布器
        self.command_pub = rospy.Publisher('/voice/recognized_command', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/voice/status', String, queue_size=10)
        
        # 串口连接
        self.ser = None
        self.connect_serial()
        
        # 状态变量
        self.last_command_time = time.time()
        self.is_listening = True
        
        rospy.loginfo("Voice Command Node initialized on port %s", self.serial_port)
        
    def create_twist(self, linear_x, angular_z):
        """创建Twist消息"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist
        
    def connect_serial(self):
        """连接串口设备"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            if self.ser.is_open:
                rospy.loginfo("Successfully connected to voice module on %s", self.serial_port)
                # 启动接收线程
                self.receive_thread = threading.Thread(target=self.receive_loop)
                self.receive_thread.daemon = True
                self.receive_thread.start()
            else:
                rospy.logerr("Failed to open serial port %s", self.serial_port)
                
        except serial.SerialException as e:
            rospy.logerr("Serial connection error: %s", e)
            self.ser = None
            
    def receive_loop(self):
        """串口数据接收循环"""
        buffer = ""
        
        while not rospy.is_shutdown() and self.ser and self.ser.is_open:
            try:
                # 读取数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # 处理完整的数据行
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self.process_voice_data(line)
                            
            except serial.SerialException as e:
                rospy.logwarn("Serial read error: %s", e)
                time.sleep(1)
            except UnicodeDecodeError as e:
                rospy.logwarn("Data decode error: %s", e)
                buffer = ""
            except Exception as e:
                rospy.logwarn("Unexpected error in receive loop: %s", e)
                
    def process_voice_data(self, data):
        """处理语音模块数据"""
        # CI1302模块通常的通信格式示例:
        # "asr:你好" 或 "wake:小智" 或 "command:前进"
        
        rospy.logdebug("Raw voice data: %s", data)
        
        # 简单的协议解析
        if data.startswith('asr:'):
            # 语音识别结果
            command = data[4:].strip()
            self.handle_voice_command(command)
            
        elif data.startswith('wake:'):
            # 唤醒词检测
            wake_word = data[5:].strip()
            rospy.loginfo("Wake word detected: %s", wake_word)
            self.publish_status("wakeup")
            
        elif data.startswith('command:'):
            # 预定义命令
            command = data[8:].strip()
            self.handle_voice_command(command)
            
        elif data.startswith('error:'):
            # 错误信息
            error_msg = data[6:].strip()
            rospy.logwarn("Voice module error: %s", error_msg)
            self.publish_status("error:" + error_msg)
            
        elif data.startswith('ready'):
            # 模块就绪
            rospy.loginfo("Voice module is ready")
            self.publish_status("ready")
            
    def handle_voice_command(self, command):
        """处理语音命令"""
        if not command:
            return
            
        rospy.loginfo("Voice command recognized: %s", command)
        
        # 发布原始命令
        cmd_msg = String()
        cmd_msg.data = command
        self.command_pub.publish(cmd_msg)
        
        # 检查命令映射
        if command in self.command_mapping:
            # 直接发布控制指令
            twist_cmd = self.command_mapping[command]
            self.cmd_vel_pub.publish(twist_cmd)
            rospy.loginfo("Executing command: %s -> linear: %.1f, angular: %.1f", 
                         command, twist_cmd.linear.x, twist_cmd.angular.z)
            self.publish_status("executing:" + command)
        else:
            # 未知命令
            rospy.logwarn("Unknown voice command: %s", command)
            self.publish_status("unknown:" + command)
            
        self.last_command_time = time.time()
        
    def publish_status(self, status):
        """发布状态信息"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        
    def send_serial_command(self, command):
        """向语音模块发送命令"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((command + '\n').encode('utf-8'))
                self.ser.flush()
            except serial.SerialException as e:
                rospy.logwarn("Failed to send command: %s", e)
                
    def configure_voice_module(self):
        """配置语音模块"""
        # 设置唤醒词
        self.send_serial_command("set_wakeword:小智")
        time.sleep(0.1)
        
        # 设置识别模式
        self.send_serial_command("set_mode:continuous")
        time.sleep(0.1)
        
        # 设置灵敏度
        self.send_serial_command("set_sensitivity:5")
        time.sleep(0.1)
        
        rospy.loginfo("Voice module configured")
        
    def run(self):
        """主运行循环"""
        # 等待串口连接
        time.sleep(2)
        
        if self.ser and self.ser.is_open:
            self.configure_voice_module()
        
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # 检查命令超时
            current_time = time.time()
            if current_time - self.last_command_time > self.command_timeout:
                # 超时后停止
                stop_cmd = self.create_twist(0.0, 0.0)
                self.cmd_vel_pub.publish(stop_cmd)
                self.last_command_time = current_time
                
            rate.sleep()
            
    def cleanup(self):
        """清理资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            rospy.loginfo("Serial port closed")

if __name__ == '__main__':
    try:
        node = VoiceCommandNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()