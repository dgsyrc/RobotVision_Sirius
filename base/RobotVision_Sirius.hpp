/**
 * @file RobotVision_Sirius.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 主函数
 * @date 2023-01-15
 * @copyright Copyright (c) 2023 Sirius
 */
#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/core.hpp>

#include "devices/camera/mv_video_capture.hpp"
#include "devices/serial/uart_serial.hpp"
#include "module/armor/basic_armor.hpp"
#include "module/buff/basic_buff.hpp"
#include "module/ml/onnx_inferring.hpp"
#include "module/record/record.hpp"
#include "module/roi/basic_roi.hpp"
#include "utils/reset_mv_camera.hpp"
#include "module/camera/camera_calibration.hpp"
#include "module/angle_solve/angle_solve.hpp"
#include "module/buff/new_buff.hpp"

#include <string>

auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "RobotVision");
time_t time_now = time(0);
bool fire;
long long rec_time=0;
int rec_cnt=0;