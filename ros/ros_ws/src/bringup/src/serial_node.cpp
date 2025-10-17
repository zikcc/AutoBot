// autobot_bringup/src/serial_node.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "protocol.h"

class SerialBridgeNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // ROS订阅器和发布器
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher imu_pub_;
    
    // 串口对象
    serial::Serial serial_port_;
    
    // 参数
    std::string serial_port_;
    int baud_rate_;
    int timeout_ms_;
    
public:
    SerialBridgeNode() : private_nh_("~") {
        // 获取参数
        private_nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
        private_nh_.param("baud_rate", baud_rate_, 115200);
        private_nh_.param("timeout_ms", timeout_ms_, 1000);
        
        // 初始化串口
        try {
            serial_port_.setPort(serial_port_);
            serial_port_.setBaudrate(baud_rate_);
            serial_port_.setTimeout(timeout_ms_, timeout_ms_, 0, timeout_ms_, 0);
            serial_port_.open();
            ROS_INFO("Successfully opened serial port: %s", serial_port_.c_str());
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to open serial port: %s", e.what());
            ros::shutdown();
        }
        
        // 初始化ROS订阅和发布
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &SerialBridgeNode::cmdVelCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_data", 50);
        
        // 启动接收线程
        receive_thread_ = std::thread(&SerialBridgeNode::receiveLoop, this);
    }
    
    ~SerialBridgeNode() {
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 构建速度指令帧
        ProtocolFrame frame;
        uint8_t payload[4];
        
        // 将速度数据转换为字节数组 (x, z)
        int16_t linear_x = static_cast<int16_t>(msg->linear.x * 1000); // m/s -> mm/s
        int16_t angular_z = static_cast<int16_t>(msg->angular.z * 1000); // rad/s -> mrad/s
        
        payload[0] = (linear_x >> 8) & 0xFF;
        payload[1] = linear_x & 0xFF;
        payload[2] = (angular_z >> 8) & 0xFF;
        payload[3] = angular_z & 0xFF;
        
        build_frame(&frame, CMD_SET_VELOCITY, payload, 4);
        
        // 发送帧
        try {
            serial_port_.write(reinterpret_cast<uint8_t*>(&frame), sizeof(frame));
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to send data: %s", e.what());
        }
    }
    
    void receiveLoop() {
        uint8_t buffer[MAX_FRAME_LEN];
        size_t bytes_read;
        
        while (ros::ok()) {
            try {
                bytes_read = serial_port_.read(buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    processReceivedData(buffer, bytes_read);
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Serial read error: %s", e.what());
            }
        }
    }
    
    void processReceivedData(const uint8_t* data, size_t length) {
        // 简化的协议解析（实际中需要更复杂的状态机）
        for (size_t i = 0; i < length; i++) {
            if (data[i] == PROTOCOL_HEADER) {
                if (i + sizeof(ProtocolFrame) <= length) {
                    ProtocolFrame* frame = (ProtocolFrame*)(data + i);
                    if (verify_frame(frame)) {
                        handleProtocolFrame(frame);
                    }
                }
            }
        }
    }
    
    void handleProtocolFrame(const ProtocolFrame* frame) {
        switch (frame->cmd_type) {
            case CMD_GET_ODOM: {
                // 解析里程计数据
                if (frame->data_len >= 12) {
                    nav_msgs::Odometry odom_msg;
                    odom_msg.header.stamp = ros::Time::now();
                    odom_msg.header.frame_id = "odom";
                    odom_msg.child_frame_id = "base_link";
                    
                    // 从payload解析位置和速度数据（示例）
                    // 实际中需要根据STM32发送的数据格式解析
                    odom_pub_.publish(odom_msg);
                }
                break;
            }
            case CMD_GET_IMU: {
                // 解析IMU数据
                if (frame->data_len >= 12) {
                    sensor_msgs::Imu imu_msg;
                    imu_msg.header.stamp = ros::Time::now();
                    imu_msg.header.frame_id = "imu_link";
                    
                    // 从payload解析IMU数据（示例）
                    // 实际中需要根据STM32发送的数据格式解析
                    imu_pub_.publish(imu_msg);
                }
                break;
            }
            case CMD_ACK:
                ROS_DEBUG("Received ACK from STM32");
                break;
        }
    }
    
private:
    std::thread receive_thread_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_bridge_node");
    SerialBridgeNode node;
    ros::spin();
    return 0;
}