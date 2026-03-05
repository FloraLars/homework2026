//
// Created by floralord on 2026/2/26.
//
#include "homework2026/aim_node.h"
#include <cmath>
#include <iostream>
#include <ostream>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "../include/homework2026/kalman.h"

namespace homework2026 {
    AimNode::AimNode() :
    Node("aim_node") ,
    //时间定为当时
    last_time_(this -> get_clock()->now()) {
        //自动找已打开的串口
        communicator_.AutoFindSerial();
        //订阅图像话题
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",10,
            std::bind(&AimNode::ImageCallback, this, std::placeholders::_1));
        pub_x_ =create_publisher<std_msgs::msg::Float32>("/kalman/x_",10);
        pub_y_ =create_publisher<std_msgs::msg::Float32>("/kalman/y_",10);
        pub_vx_ =create_publisher<std_msgs::msg::Float32>("/kalman/vx_",10);
    }
    void AimNode::ImageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg) {
        //算时差
        auto current_time = this->get_clock()->now();
        float dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        //把ROS图像转化为OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,msg->encoding);
        cv::Mat img = cv_ptr->image;
        //判断己方颜色
        if (self_color_ == ColorType::kUnknown) {
            self_color_ = detector_.DetectSelfColor(img);
        }
        //找敌方目标
        auto enemies = detector_.DetectTargets(img,self_color_);
        auto target = detector_.GetTarget(enemies);
        //若目标有效进入卡尔曼滤波
        if (target.is_valid) {
            float x,y,vx;
            if (!has_target_) {
                kalman_.Init(target.center_x, target.center_y);
                has_target_ = true;
                x = target.center_x;
                y = target.center_y;
                vx = 0;
            }else {
                kalman_.Predict(dt);
                kalman_.Update(target.center_x, target.center_y);
                x = kalman_.GetX();
                y = kalman_.GetY();
                vx = kalman_.GetVx();
            }
            //发布状态量
            std_msgs::msg::Float32 msg_x,msg_y, msg_vx;
            msg_x.data = x;
            msg_y.data = y;
            msg_vx.data = vx;
            pub_x_ ->publish(msg_x);
            pub_y_ ->publish(msg_y);
            pub_vx_ ->publish(msg_vx);
            //子弹600像素/秒，炮台中心（576,600）
            const float BULLET_SPEED = 600.0;
            const float TURRET_X = 576.0;
            const float TURRET_Y = 648.0;
            //算目标和炮台距离
            float dx = x - TURRET_X;
            float dy = TURRET_Y - y;
            float dist = sqrt(dx * dx + dy * dy);
            //算子弹飞行时间
            float t_flight = dist / BULLET_SPEED;
            //预测位置
            float x_future =x + (0+t_flight)* vx;
            float y_future =y;
            //算角度
            float angle = communicator_.CalculateAngle(x_future, y_future);
            //发角度数据
            communicator_.SendAngle(angle);
            //连开三次
            for (int i = 0; i < 3; i++) {
                communicator_.SendFire();
            usleep(10000);
            }
        }else {
            kalman_.Predict(dt);
            has_target_ = false;
        }
    }
}
using namespace homework2026;
int main(int argc, char** argv) {
    //初始化ROS2运行环境
    rclcpp::init(argc, argv);
    //实例化节点
    auto node = std::make_shared<AimNode>();
    //启动节点事件循环，等回调
    rclcpp::spin(node);
    //关闭ROS2环境，释放资源
    rclcpp::shutdown();
    //退出
    return 0;
}