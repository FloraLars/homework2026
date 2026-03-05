//
// Created by floralord on 2026/2/25.
//
#include "../include/homework2026/communicator.h"
#include "homework2026/communicator.h"
#include <cstdint>
#include <cstring>
#include <cmath>
#include <dirent.h>
#include <cctype>
namespace homework2026 {
    Communicator::Communicator() {}
    //打开串口
    bool Communicator::OpenSerial(const std::string &port) {
        if (serial_.IsOpen()) {
            serial_.Close();
        }
        serial_.SetPort(port);
        serial_.SetBaudRate(BAUD_RATE);
        serial_.Open();
        return serial_.IsOpen();
    }
    //打开文件自动寻找串口并打开
    bool Communicator::AutoFindSerial() {
        //打开目录寻找纯数字的包名
        DIR* dir = opendir("/dev/pts");
        if (dir == nullptr) {return false;}
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            const char* name = entry->d_name;
            bool is_number = true;
            for (int i=0; name[i]!='\0'; i++) {
                if (!isdigit(name[i])) {
                    is_number = false;
                    break;
                }
            }
            if (!is_number) continue;
            //拼接出完整串口名并关闭打开的文件夹
            std::string port = "/dev/pts/";
            port += name;
            if (OpenSerial(port)) {
                closedir(dir);
                return true;
            }
        }
        closedir(dir);
        return false;
    }
    //计算角度函数
    float Communicator::CalculateAngle(float target_x , float target_y) const {
        float dx = target_x - kX;
        float dy = kY - target_y;
        float angle = atan2(dy, dx) * 180 / M_PI;
        if (angle < -180.0) angle += 360.0;
        if (angle > 180.0) angle -= 360.0;
        return angle;
    }
    //发送角度通讯函数
    void Communicator::SendAngle(float angle) {
        uint8_t cmd[5]={0x01};
        std::memcpy(&cmd[1], &angle, sizeof(angle));
        if (serial_.IsOpen()) {
            serial_.write(cmd,5);
        }
    }
    //发送开火信息函数
    void Communicator::SendFire() {
        uint8_t cmd = 0x02;
        if (serial_.IsOpen()) {
                serial_.write(&cmd,1);
        }
    }
}