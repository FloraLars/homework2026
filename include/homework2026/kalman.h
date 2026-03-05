//
// Created by floralord on 2026/2/25.
//

#ifndef ROS2_WORKPLACE2_KALMAN_H
#define ROS2_WORKPLACE2_KALMAN_H
#include <Eigen/Dense>
namespace homework2026 {
    class Kalman {
        public:
        Kalman();
        //初始化
        void Init(float init_x, float init_y);
        //预测dt后各值
        void Predict(float dt);
        //根据实际观察更新值
        void Update(float obs_x, float obs_y);
        //分别得到X、Y、Vx 、Vy的值用于运算
        float GetX() const;
        float GetY() const;
        float GetVx() const;
        private:
        //初始值
        float x_=0.0;
        float y_=0.0;
        float vx_=0.0;
        //状态向量x_,y_,vx_
        Eigen::Vector3f X_;
        //状态转移矩阵
        Eigen::Matrix3f F_;
        //观测矩阵
        Eigen::Matrix<float,2,3>H_;
        //误差协方差矩阵
        Eigen::Matrix3f P_;
        //过程噪声协方差矩阵
        Eigen::Matrix3f Q_;
        //观测噪声协方差矩阵
        Eigen::Matrix2f R_;
    };
}

#endif //ROS2_WORKPLACE2_KALMAN_H