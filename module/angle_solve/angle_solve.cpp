/**
 * @file angle_solve.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 角度解算
 * @date 2023-02-07
 * @copyright Copyright (c) 2023 Sirius
 *
 */

#include "angle_solve.hpp"

namespace angle_solve {
    void solve::set_config(std::string config_path) {
        cv::FileStorage conf(config_path, cv::FileStorage::READ);
        conf["ARMOR_HEIGHT"] >> config.armor_height;
        conf["ARMOR_LENGHT"] >> config.armor_lenght;
        conf["PIC_ARMOR_HEIGHT"] >> config.pic_armor_height;
        conf["PIC_ARMOR_LENGHT"] >> config.pic_armor_lenght;
        conf["PIC_DISTANCE"] >> config.pic_distance;
        conf["ARMOR_DISTANCE"] >> config.armor_distance;
        fmt::print("[angle info] config st {}\n",config.pic_distance);
        conf.release();
    }

    void solve::angleSolve(cv::RotatedRect object, int row, int col, uart::SerialPort& info) {
        //
        // 左加右减 上减下加
        fmt::print("[angle info] config {}\n",config.pic_distance);
        target.predict.y = config.pic_distance * config.armor_height / object.size.height;
        target.predict.x = (object.center.x - col / 2.0) * target.predict.y / config.pic_distance;
        target.predict.z = ((row - object.center.y) - row / 2.0) * target.predict.y / config.pic_distance;
        fmt::print("[{}] L: {}  H: {} row: {} object_y: {}\n", angle_info,target.predict.y, target.predict.z, row, object.center.y);
        target.time = target.predict.y / (20.5 * cos(-info.returnReceivePitch()/180*PI) * 100);
        target.predict.z = target.predict.z + 0.5 * 9.8 * 100 * target.time * target.time;
        fmt::print("[angle info] p {} {} {}\n", 0.5 * 9.8 * 100 * target.time * target.time,target.time, info.returnReceiveBulletVelocity());
        //compensation.pitch = 

        target.yaw = atan(-target.predict.x / target.predict.y) / PI * 180;
        target.pitch = atan(-target.predict.z / sqrt(pow(target.predict.y, 2) + pow(target.predict.x, 2))) / PI * 180;
        fmt::print("[angle info] x {} y {} z {} yaw {} pitch {}\n", target.predict.x, target.predict.y, target.predict.z, target.yaw, target.pitch);
        fmt::print("[angle info] y {}\n", target.predict.y);
       
        fmt::print("[angle info] center x {} center y {}\n", object.center.x, object.center.y);
    }


    float solve::returnYawAngle() {
        if(fabs(target.yaw) < 0.01) {
            return 0;

            //return target.yaw;
        } else {
            return target.yaw;
        }
        
    }

    float solve::returnPitchAngle() {
        if(fabs(target.pitch) < 0.01) {
            return 0;
            //return target.pitch;
        } else {
            return target.pitch;
        }
        
    }
} // namespace angle_solve
