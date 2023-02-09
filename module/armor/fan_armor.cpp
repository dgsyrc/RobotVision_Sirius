/**
 * @file fan_armor.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 扇叶装甲板类
 * @date 2023-01-18
 * 
 * @copyright Copyright (c) 2023 Sirius
 * 
 */

#include "fan_armor.hpp"

namespace fan_armor {
Detector::Detector() {}

Detector::~Detector() {}

void Detector::inputParams(const std::vector<cv::Point>& _contours) { abstract_object::Object::inputParams(_contours); }

void Detector::displayFanArmor(cv::Mat& _img) {
  cv::Scalar color = cv::Scalar(0, 255, 0);

  cv::Point put_fan_armor = cv::Point(_img.cols - 100, 10);
  cv::putText(_img, " Fan Armor ", put_fan_armor, cv::FONT_HERSHEY_PLAIN, 1, color, 1, 8, false);

  for (int k = 0; k < 4; ++k) {
    cv::line(_img, vertex_[k], vertex_[(k + 1) % 4], color, 2);
  }
}

}  // namespace fan_armor
