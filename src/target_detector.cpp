//
// Created by floralord on 2026/2/25.
//
#include "homework2026/target_detector.h"
#include <cmath>
#include <iostream>
#include <ostream>

namespace homework2026 {
    //找自己颜色
    ColorType TargetDetector::DetectSelfColor(const cv::Mat& image) {
        int w=image.cols;
        int h=image.rows;
        cv::Rect rect(w/2-15,h-20,30,20);
        cv::Mat hsv;
        //转化图像判断
        cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);
        cv::Mat blue_mask,red_mask,mask1,mask2;
        //找蓝色
        cv::inRange(hsv, cv::Scalar(90,100,100),cv::Scalar(124,255,255),blue_mask);
        //找红色
        cv::inRange(hsv, cv::Scalar(0,120,70),cv::Scalar(10,255,255),mask1);
        cv::inRange(hsv, cv::Scalar(160,120,70),cv::Scalar(180,255,255),mask2);
        cv::bitwise_or(mask1, mask2, red_mask);
        //把找到的红蓝图转化成像素个数
        int blue = cv::countNonZero(blue_mask);
        int red = cv::countNonZero(red_mask);
        //比较得出自己的颜色
        return (blue > red) ? ColorType::kBlue : ColorType::kRed;
    }
    //辅助函数1：找灯条，输入图片和敌方颜色找敌方灯条
    std::vector<cv::Rect> TargetDetector::FindLightRects(const cv::Mat& image,ColorType color) {
        cv::Mat hsv,mask,mask1,mask2;
        cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);
        if (color == ColorType::kBlue) {
            cv::inRange(hsv, cv::Scalar(90,100,100),cv::Scalar(124,255,255),mask);
        }else if (color == ColorType::kRed) {
            cv::inRange(hsv, cv::Scalar(0,120,70),cv::Scalar(10,255,255),mask1);
            cv::inRange(hsv, cv::Scalar(160,120,70),cv::Scalar(180,255,255),mask2);
            cv::bitwise_or(mask1, mask2, mask);
        }else {
            return {};
        }
        //储存找到的所有灯条，为下一步两两配对做准备
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::Rect> rects;
        for (auto& contour : contours) {
            cv::Rect rect = cv::boundingRect(contour);
            if (rect.height > rect.width) {
                rects.push_back(rect);
            }
        }
        return rects;
    }
    //辅助函数2：灯条配对，找到装甲板和其中心
    std::vector<Target> TargetDetector::MatchLightRects(const std::vector<cv::Rect>& bars) {
        std::vector<Target> targets;
        //遍历配对
        for (size_t i = 0; i < bars.size(); i++) {
            for (size_t j = i+1 ; j < bars.size(); j++) {
                const cv::Rect&bar1 = bars[i];
                const cv::Rect&bar2 = bars[j];
                //不符合要求就跳过
                if (std::abs(bar1.y - bar2.y) > 5) continue;
                int width =abs( bar2.x - (bar1.x+bar1.width));
                if (width < 15 || width > 80) continue;
                Target target;
                target.center_x = (bar1.x + bar1.width/2 + bar2.x + bar2.width/2)/2.0;
                target.center_y = (bar1.y + bar1.height/2 + bar2.height/2 + bar2.y)/2.0;
                target.is_valid = true;
                //找到目标放进去
                targets.push_back(target);
            }
        }

        return targets;
    }
    //根据两个辅助函数找目标（多个）
    std::vector<Target> TargetDetector::DetectTargets(const cv::Mat& image,ColorType self_color) {
        ColorType enemy_color = (self_color == ColorType::kBlue) ? ColorType::kRed : ColorType::kBlue;
        auto rects = FindLightRects(image,enemy_color);
        return MatchLightRects(rects);
    }
    //找到开火目标
    Target TargetDetector::GetTarget(const std::vector<Target>& targets) {
        for (auto& target : targets) {
            if (target.is_valid) {
                return target;
            }
        }
        return {};
    }
}
