//
// Created by floralord on 2026/2/25.
//

#ifndef ROS2_WORKPLACE2_COMMUNICATOR_H
#define ROS2_WORKPLACE2_COMMUNICATOR_H
#include <string>
#include <cstdint>
#include <dirent.h>
#include <cctype>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
namespace serial {
    //造一个通讯类，实现两者通讯
    class Serial {
        public:
        Serial(): fd_(-1) {}
        //设置串口
        void SetPort(const std::string& port){port_=port;}
        //设置波特率
        void SetBaudRate(int){}
        //打开串口
        bool Open() {
            fd_=open(port_.c_str(),O_RDWR | O_NOCTTY);
            return fd_!=-1;
        }
        //判断串口是否已经打开
        bool IsOpen() const {return fd_!=-1;}
        //关闭串口
        void Close() {
            if (fd_!=1) {
                ::close(fd_);
                fd_=-1;
            }
        }
        //发送数据到串口
        ssize_t write(const uint8_t* data,size_t len) {
            return ::write(fd_,data,len);
        }
    private:
        int fd_;
        std::string port_;
    };
}
namespace homework2026 {
    class Communicator {
        public:
        Communicator();
        //开串口
        bool OpenSerial(const std::string & port);
        //自识别串口
        bool AutoFindSerial();
        //角度计算函数
        float CalculateAngle(float target_x, float target_y)const;
        //发送角度
        void SendAngle(float angle);
        //发送开火信息
        void SendFire();
        private:
        static const int BAUD_RATE = 115200;
        serial::Serial serial_;
        const float kX = 576.0;
        const float kY = 648.0;
    };
}
#endif //ROS2_WORKPLACE2_COMMUNICATOR_H