#!/usr/bin/env python
# autobot_voice/scripts/train_commands.py
import rospy
import serial
import time
import json
from std_msgs.msg import String

class VoiceTrainer:
    def __init__(self):
        rospy.init_node('voice_trainer', anonymous=True)
        
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        
        # 命令配置文件
        self.commands_file = rospy.get_param('~commands_file', 'voice_commands.json')
        
        self.ser = None
        self.connect_serial()
        
    def connect_serial(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            rospy.loginfo("Connected to voice module for training")
        except serial.SerialException as e:
            rospy.logerr("Failed to connect: %s", e)
            return False
        return True
        
    def send_command(self, command):
        """发送命令到模块"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((command + '\n').encode('utf-8'))
                time.sleep(0.1)
                response = self.ser.readline().decode('utf-8').strip()
                return response
            except Exception as e:
                rospy.logerr("Send command error: %s", e)
        return None
        
    def load_commands(self):
        """加载命令配置"""
        try:
            with open(self.commands_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            rospy.logwarn("Commands file not found, using default commands")
            return self.get_default_commands()
            
    def get_default_commands(self):
        """获取默认命令集"""
        return {
            "wakewords": ["小智", "robot", "hey bot"],
            "commands": {
                "前进": "forward",
                "后退": "backward", 
                "左转": "turn left",
                "右转": "turn right",
                "停止": "stop",
                "加速": "faster",
                "减速": "slower"
            }
        }
        
    def train_wakewords(self, wakewords):
        """训练唤醒词"""
        rospy.loginfo("Training wakewords...")
        
        for wakeword in wakewords:
            rospy.loginfo("Please say wakeword: '%s' 3 times", wakeword)
            
            # 进入训练模式
            response = self.send_command("train_wakeword:" + wakeword)
            if response != "ready":
                rospy.logerror("Failed to start wakeword training")
                return False
                
            # 等待训练完成
            trained = False
            for _ in range(10):  # 最多等待10秒
                response = self.send_command("check_training")
                if response == "trained":
                    trained = True
                    break
                time.sleep(1)
                
            if trained:
                rospy.loginfo("Wakeword '%s' trained successfully", wakeword)
            else:
                rospy.logerror("Failed to train wakeword '%s'", wakeword)
                return False
                
        return True
        
    def train_commands(self, commands):
        """训练命令词"""
        rospy.loginfo("Training commands...")
        
        for chinese, english in commands.items():
            rospy.loginfo("Please say command: '%s' (%s) 3 times", chinese, english)
            
            # 进入训练模式
            response = self.send_command("train_command:" + chinese + ":" + english)
            if response != "ready":
                rospy.logerror("Failed to start command training")
                return False
                
            # 等待训练完成
            trained = False
            for _ in range(10):
                response = self.send_command("check_training")
                if response == "trained":
                    trained = True
                    break
                time.sleep(1)
                
            if trained:
                rospy.loginfo("Command '%s' trained successfully", chinese)
            else:
                rospy.logerror("Failed to train command '%s'", chinese)
                return False
                
        return True
        
    def save_configuration(self):
        """保存配置到模块"""
        response = self.send_command("save_config")
        if response == "saved":
            rospy.loginfo("Configuration saved to module")
            return True
        else:
            rospy.logerror("Failed to save configuration")
            return False
            
    def run_training(self):
        """运行训练流程"""
        if not self.ser or not self.ser.is_open:
            rospy.logerror("Not connected to voice module")
            return False
            
        # 加载命令配置
        config = self.load_commands()
        
        # 训练唤醒词
        if not self.train_wakewords(config["wakewords"]):
            return False
            
        # 训练命令词
        if not self.train_commands(config["commands"]):
            return False
            
        # 保存配置
        if not self.save_configuration():
            return False
            
        rospy.loginfo("Voice training completed successfully!")
        return True
        
    def cleanup(self):
        """清理资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    trainer = VoiceTrainer()
    
    try:
        if trainer.run_training():
            rospy.loginfo("Training completed successfully!")
        else:
            rospy.logerr("Training failed!")
    except KeyboardInterrupt:
        rospy.loginfo("Training interrupted by user")
    finally:
        trainer.cleanup()