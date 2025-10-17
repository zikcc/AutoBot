// autobot_control/src/motion_controller.cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h> // 如果用深度相机生成激光扫描数据
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class MotionController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布器和订阅器
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber voice_cmd_sub_;
    ros::Subscriber odom_sub_;
    
    // 控制参数
    double max_linear_speed_;
    double max_angular_speed_;
    double safety_distance_;
    double control_rate_;
    
    // 状态变量
    std::string current_voice_command_;
    bool obstacle_detected_;
    double obstacle_distance_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_velocity_;
    
    // PID控制参数（简单的速度控制PID）
    struct {
        double kp_linear;
        double ki_linear;
        double kd_linear;
        double kp_angular;
        double ki_angular;
        double kd_angular;
        double integral_linear;
        double integral_angular;
        double prev_error_linear;
        double prev_error_angular;
    } pid_;

public:
    MotionController() : private_nh_("~"), 
                        obstacle_detected_(false),
                        obstacle_distance_(10.0) { // 初始设为较远距离
        
        // 加载参数
        private_nh_.param("max_linear_speed", max_linear_speed_, 0.5);
        private_nh_.param("max_angular_speed", max_angular_speed_, 1.0);
        private_nh_.param("safety_distance", safety_distance_, 0.5);
        private_nh_.param("control_rate", control_rate_, 10.0);
        
        // PID参数
        private_nh_.param("pid_linear/kp", pid_.kp_linear, 0.8);
        private_nh_.param("pid_linear/ki", pid_.ki_linear, 0.01);
        private_nh_.param("pid_linear/kd", pid_.kd_linear, 0.05);
        private_nh_.param("pid_angular/kp", pid_.kp_angular, 1.2);
        private_nh_.param("pid_angular/ki", pid_.ki_angular, 0.02);
        private_nh_.param("pid_angular/kd", pid_.kd_angular, 0.1);
        
        // 初始化PID状态
        pid_.integral_linear = 0;
        pid_.integral_angular = 0;
        pid_.prev_error_linear = 0;
        pid_.prev_error_angular = 0;
        
        // 发布控制指令
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        
        // 订阅各种传感器和指令信息
        depth_image_sub_ = nh_.subscribe("depth_image", 1, &MotionController::depthImageCallback, this);
        voice_cmd_sub_ = nh_.subscribe("voice_commands", 1, &MotionController::voiceCommandCallback, this);
        odom_sub_ = nh_.subscribe("odom", 1, &MotionController::odomCallback, this);
        
        ROS_INFO("Motion Controller initialized");
        ROS_INFO("Max linear speed: %.2f m/s, Max angular speed: %.2f rad/s", 
                 max_linear_speed_, max_angular_speed_);
    }
    
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // 简化的障碍物检测：计算深度图像中最近的距离
        // 实际应用中应该使用更复杂的计算机视觉算法
        try {
            // 这里应该是实际的深度图像处理逻辑
            // 简化版：假设从其他节点已经处理好了障碍物信息
            // 我们订阅一个更简单的障碍物距离话题
        } catch (const std::exception& e) {
            ROS_WARN("Depth image processing error: %s", e.what());
        }
    }
    
    // 订阅处理过的障碍物信息（来自autobot_vision包）
    void obstacleCallback(const sensor_msgs::LaserScanConstPtr& scan) {
        obstacle_detected_ = false;
        obstacle_distance_ = 10.0; // 初始设为较大值
        
        // 检查扫描数据中的最小距离
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (std::isfinite(scan->ranges[i]) && scan->ranges[i] < obstacle_distance_) {
                obstacle_distance_ = scan->ranges[i];
            }
        }
        
        // 如果障碍物太近，标记检测到障碍物
        if (obstacle_distance_ < safety_distance_) {
            obstacle_detected_ = true;
            ROS_WARN("Obstacle detected at distance: %.2f m", obstacle_distance_);
        }
    }
    
    void voiceCommandCallback(const std_msgs::StringConstPtr& msg) {
        current_voice_command_ = msg->data;
        ROS_INFO("Received voice command: %s", current_voice_command_.c_str());
        
        // 处理语音指令
        handleVoiceCommand(current_voice_command_);
    }
    
    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        current_pose_ = msg->pose.pose;
        current_velocity_ = msg->twist.twist;
    }
    
    void handleVoiceCommand(const std::string& command) {
        geometry_msgs::Twist cmd_vel;
        
        if (command == "前进" || command == "forward") {
            if (!obstacle_detected_) {
                cmd_vel.linear.x = max_linear_speed_;
                ROS_INFO("Executing: Move forward");
            } else {
                ROS_WARN("Cannot move forward: obstacle detected");
                cmd_vel.linear.x = 0;
            }
        } 
        else if (command == "后退" || command == "backward") {
            cmd_vel.linear.x = -max_linear_speed_ * 0.5; // 后退速度较慢
            ROS_INFO("Executing: Move backward");
        }
        else if (command == "左转" || command == "turn left") {
            cmd_vel.angular.z = max_angular_speed_;
            ROS_INFO("Executing: Turn left");
        }
        else if (command == "右转" || command == "turn right") {
            cmd_vel.angular.z = -max_angular_speed_;
            ROS_INFO("Executing: Turn right");
        }
        else if (command == "停止" || command == "stop") {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            ROS_INFO("Executing: Stop");
        }
        else {
            ROS_WARN("Unknown voice command: %s", command.c_str());
            return;
        }
        
        // 发布控制指令
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    // 自主避障导航
    void autonomousNavigation() {
        geometry_msgs::Twist cmd_vel;
        
        if (obstacle_detected_) {
            // 障碍物避让逻辑
            if (obstacle_distance_ < safety_distance_ * 0.5) {
                // 紧急停止并后退
                cmd_vel.linear.x = -0.1;
                cmd_vel.angular.z = 0.5; // 尝试转向
            } else {
                // 减速并转向避让
                cmd_vel.linear.x = max_linear_speed_ * 0.3;
                cmd_vel.angular.z = max_angular_speed_;
            }
        } else {
            // 无障碍物，正常前进
            cmd_vel.linear.x = max_linear_speed_;
            cmd_vel.angular.z = 0;
        }
        
        // 应用PID控制平滑速度变化
        cmd_vel = applyPIDSmoothing(cmd_vel);
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    // PID速度平滑控制
    geometry_msgs::Twist applyPIDSmoothing(const geometry_msgs::Twist& target_vel) {
        geometry_msgs::Twist smoothed_vel;
        double dt = 1.0 / control_rate_;
        
        // 线性速度PID
        double error_linear = target_vel.linear.x - current_velocity_.linear.x;
        pid_.integral_linear += error_linear * dt;
        double derivative_linear = (error_linear - pid_.prev_error_linear) / dt;
        
        smoothed_vel.linear.x = current_velocity_.linear.x + 
                               pid_.kp_linear * error_linear +
                               pid_.ki_linear * pid_.integral_linear +
                               pid_.kd_linear * derivative_linear;
        
        // 角速度PID
        double error_angular = target_vel.angular.z - current_velocity_.angular.z;
        pid_.integral_angular += error_angular * dt;
        double derivative_angular = (error_angular - pid_.prev_error_angular) / dt;
        
        smoothed_vel.angular.z = current_velocity_.angular.z +
                                pid_.kp_angular * error_angular +
                                pid_.ki_angular * pid_.integral_angular +
                                pid_.kd_angular * derivative_angular;
        
        // 限制速度范围
        smoothed_vel.linear.x = std::max(-max_linear_speed_, 
                                        std::min(max_linear_speed_, smoothed_vel.linear.x));
        smoothed_vel.angular.z = std::max(-max_angular_speed_,
                                         std::min(max_angular_speed_, smoothed_vel.angular.z));
        
        // 更新误差记录
        pid_.prev_error_linear = error_linear;
        pid_.prev_error_angular = error_angular;
        
        return smoothed_vel;
    }
    
    void run() {
        ros::Rate rate(control_rate_);
        
        while (ros::ok()) {
            // 如果没有语音指令控制，执行自主导航
            if (current_voice_command_.empty()) {
                autonomousNavigation();
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_controller");
    MotionController controller;
    controller.run();
    return 0;
}