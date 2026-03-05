//
// Created by floralord on 2026/2/26.
//

#ifndef ROS2_WORKPLACE2_AIM_NODE_H
#define ROS2_WORKPLACE2_AIM_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include "target_detector.h"
#include "kalman.h"
#include "communicator.h"
#include <std_msgs/msg/float32.hpp>
namespace homework2026 {
    class AimNode : public rclcpp::Node {
        public:
        AimNode();
        private:
        //图像数据回调
        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        //图像订阅
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        //发布状态量
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_x_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_y_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_vx_;
        //目标检测模块
        TargetDetector detector_;
        //卡尔曼滤波模块
        Kalman kalman_;
        //通信模块
        Communicator communicator_;
        //自身颜色初始化为未知
        ColorType self_color_ = ColorType::kUnknown;
        //是否检测到目标
        bool has_target_ = false;
        //上一次时间
        rclcpp::Time last_time_;
    };
}
#endif //ROS2_WORKPLACE2_AIM_NODE_H