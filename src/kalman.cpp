//
// Created by floralord on 2026/2/25.
//
#include "homework2026/kalman.h"

#include <asm-generic/errno.h>

namespace homework2026 {
    Kalman::Kalman() {
        //状态转移矩阵初始化为单位矩阵
        F_.setIdentity();
        //观测矩阵初始化为零矩阵
        H_.setZero();
        //协方差矩阵初始化为单位矩阵
        P_.setIdentity();
        //过程噪声协方差矩阵初始化为单位矩阵
        Q_.setIdentity();
        //观测噪声协方差矩阵初始化为单位矩阵
        R_.setIdentity();
    }
    //初始化
    void Kalman::Init(float x_init, float y_init) {
       X_<<x_init,y_init,0;
        //观测矩阵只观测x,y
        H_<<1,0,0,
            0,1,0;
        Q_<<5,0,0,
            0,0.5,0,
            0,0,5;
        R_<<2,0,
            0,2;
        P_<<1000,0,0,
            0,1000,0,
            0,0,1000;
    }
    void Kalman::Predict(float dt) {
        //赋值状态转移矩阵
        F_<<1,0,dt,
            0,1,0,
            0,0,1;
        X_ = F_*X_;
        //预测更新P矩阵
        P_=F_*P_*F_.transpose()+Q_;
        x_=X_(0);
        y_=X_(1);
        vx_=X_(2);
    }
    void Kalman::Update(float obs_x, float obs_y) {
        Eigen::Vector2f Z;
        Z<<obs_x,obs_y;
        Eigen::Vector2f y=Z-H_*X_;
        //计算残差协方差
        Eigen::Matrix2f S = H_*P_*H_.transpose()+R_;
        //计算卡尔曼增益
        Eigen::Matrix<float,3,2>K=P_*H_.transpose()*S.inverse();
        X_=X_+K*y;
        //更新P
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        P_=(I-K*H_)*P_;
        x_=X_(0);
        y_=X_(1);
        vx_=X_(2);
    }
    //取值函数
    float Kalman::GetX() const {return x_;}
    float Kalman::GetY() const {return y_;}
    float Kalman::GetVx() const {return vx_;}
}
