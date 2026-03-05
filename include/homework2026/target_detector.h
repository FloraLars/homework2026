//
// Created by floralord on 2026/2/25.
//

#ifndef ROS2_WORKPLACE2_TARGET_DETECTOR_H
#define ROS2_WORKPLACE2_TARGET_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
namespace homework2026 {
//枚举颜色
    enum class ColorType{
        kRed=0,
        kBlue=1,
        kUnknown=-1
    };
//设置目标应有特征结构
    struct Target {
        float center_x = 0.0;
        float center_y = 0.0;
        bool is_valid = false;
    };

    class TargetDetector {
        public:
        //先识别自己颜色
        ColorType DetectSelfColor(const cv::Mat& image);
        //找目标（多个）
        std::vector<Target> DetectTargets(const cv::Mat& image,ColorType self_type);
        //找一个开火目标
        Target GetTarget(const std::vector<Target>& targets);

        private:
        std::vector<cv::Rect> FindLightRects(const cv::Mat& image,ColorType color);
        std::vector<Target> MatchLightRects(const std::vector<cv::Rect>& bars);
    };
}
#endif //ROS2_WORKPLACE2_TARGET_DETECTOR_H