/**
 * @file new_buff.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 新能量机关检测
 * @date 2023-01-15
 * 
 * @copyright Copyright (c) 2023 Sirius
 * 
 */

#pragma once

#define PI 3.14159265

#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include "utils/fps.hpp"




namespace new_buff {
    /***
     * @brief 检测模式
     */
    cv::Mat ele_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));  // 椭圆内核
    enum Check_Moudle {
        ACTION_MODE,
        INACTION_MODE,
        CIRCLE_MODE,
    };

    cv::Point2f armor_last;
    cv::Point2f armor_now;
    cv::Point2f circles;

    fps::FPS new_buff_fps;

    class buff {
        public:

            enum predict_status {
                MAX,
                MIN,
                NONE,
            };
            void getcontour(cv::Mat imgDil,cv::Mat img, cv::Mat& img_src,new_buff::Check_Moudle moudle);

            void set_config(std::string config_path);

            void predict(cv::Point2f &circle_r);

            cv::Point2f calculateCord(cv::Point2f &circle_r);

            void main_buff_checker(cv::Mat img, cv::Mat img_src,new_buff::Check_Moudle moudle);
        private:

            struct info {
                float armor_height;
                float armor_lenght;
                float pic_armor_height;
                float pic_armor_lenght;
                float pic_distance;
                float armor_distance;
            } config;

            
            predict_status status = NONE;

            float last_velocity= 0.0;
            float now_velocity = 0.0;
            float min_velocity = 0.0;
            float max_velocity = 0.0;

            double timex;
            double last_timex;
            float T;

            bool start_T = false; // 检测到第一个最值标志

            cv::Point2f relay;
            bool first_T_finish = false;
            float arg[3]= {0};
            float alpha = 0.0;
            float omega = 0.0;
            float max_v = 0.0;
            float min_v = 0.0;
            float d_t;
            float velocity;
            float A = 1.0;

            float R;

            int sign = 1;

            float beta = 0.0;
            cv::Point2f object;
    };

}




