/**
 * @file record.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 视频保存
 * @date 2023-01-18
 * @copyright Copyright (c) 2023 Sirius
 *
 */
#include "record.hpp"

#include <iostream>

namespace RecordMode
{
    Record::Record() {}
    Record::Record(std::string path_input, std::string path_in, cv::Size size)
    {
        video_save_path_ = path_input;
    }
    Record::~Record() {}

} // namespace RecordMode
