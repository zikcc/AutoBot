// autobot_vision/src/depth_filter.cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class DepthFilter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher filtered_pub_;
    
    // 滤波参数
    int bilateral_d_;
    double bilateral_sigma_color_;
    double bilateral_sigma_space_;
    int median_kernel_size_;
    int gaussian_kernel_size_;
    
public:
    DepthFilter() : private_nh_("~"), it_(nh_) {
        // 加载滤波参数
        private_nh_.param("bilateral_d", bilateral_d_, 5);
        private_nh_.param("bilateral_sigma_color", bilateral_sigma_color_, 50.0);
        private_nh_.param("bilateral_sigma_space", bilateral_sigma_space_, 50.0);
        private_nh_.param("median_kernel_size", median_kernel_size_, 3);
        private_nh_.param("gaussian_kernel_size", gaussian_kernel_size_, 3);
        
        // 订阅和发布
        depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1, 
                                  &DepthFilter::depthCallback, this);
        filtered_pub_ = it_.advertise("/camera/depth/filtered", 1);
        
        ROS_INFO("Depth Filter Node initialized");
    }
    
    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 转换深度图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            
            // 应用滤波处理
            cv::Mat filtered = applyFilters(cv_ptr->image);
            
            // 发布滤波后的图像
            cv_bridge::CvImage out_msg;
            out_msg.header = msg->header;
            out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            out_msg.image = filtered;
            filtered_pub_.publish(out_msg.toImageMsg());
            
        } catch (const std::exception& e) {
            ROS_ERROR("Depth filter error: %s", e.what());
        }
    }
    
    cv::Mat applyFilters(cv::Mat depth_image) {
        cv::Mat filtered = depth_image.clone();
        
        // 1. 中值滤波（去除椒盐噪声）
        if (median_kernel_size_ > 0 && median_kernel_size_ % 2 == 1) {
            cv::medianBlur(depth_image, filtered, median_kernel_size_);
        }
        
        // 2. 双边滤波（保边去噪）
        if (bilateral_d_ > 0) {
            cv::bilateralFilter(filtered, filtered, bilateral_d_, 
                               bilateral_sigma_color_, bilateral_sigma_space_);
        }
        
        // 3. 高斯模糊（进一步平滑）
        if (gaussian_kernel_size_ > 0 && gaussian_kernel_size_ % 2 == 1) {
            cv::GaussianBlur(filtered, filtered, 
                           cv::Size(gaussian_kernel_size_, gaussian_kernel_size_), 0);
        }
        
        // 4. 无效值填充（使用邻域均值）
        fillInvalidPixels(filtered);
        
        return filtered;
    }
    
    void fillInvalidPixels(cv::Mat& image) {
        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        
        // 创建无效像素掩码（NaN, Inf, 0值）
        for (int i = 0; i < image.rows; i++) {
            for (int j = 0; j < image.cols; j++) {
                float val = image.at<float>(i, j);
                if (std::isnan(val) || std::isinf(val) || val == 0) {
                    mask.at<uchar>(i, j) = 1;
                }
            }
        }
        
        // 使用邻域均值填充无效像素
        cv::Mat filled;
        cv::inpaint(image, mask, filled, 3, cv::INPAINT_TELEA);
        filled.copyTo(image, mask);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_filter");
    DepthFilter filter;
    ros::spin();
    return 0;
}