/**
 * @file camera_calibration.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 相机标定
 * @date 2023-01-31
 * @copyright Copyright (c) 2023 Sirius
 */

#pragma once

#include <cstring>
#include <vector>

#include <opencv4/opencv2/opencv.hpp>

#include <fmt/color.h>
#include <fmt/core.h>

namespace cam{

cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
cv::Mat distCoeffs;
cv::Mat img;
cv::Mat gray;
cv::Mat rectify;
cv::Size s;

std::vector<cv::Mat> rvecs;
std::vector<cv::Mat> tvecs;
std::vector<cv::Point2f> corners;
std::vector<std::vector<cv::Point2f>> corners_;
std::vector<cv::Point3f> obj;
std::vector<std::string> files;

std::vector<std::vector<cv::Point2f>> imagePoints;
std::vector<std::vector<cv::Point3f>> objectPoints;

cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

int numCornersHor = 7;
int numCornersVer = 7;
int numSquares = 50;

int index = 0; // 网格拍照计数

bool ret;

char ch;


/**
* @brief 获取标定用图（手动）
* 
* @param frame 当前帧
*/
void create_images(cv::Mat& frame);

/**
* @brief 获取标定用图（自动）
* 
* @param image 当前帧
*/
void auto_create_images(cv::Mat& image);

/**
* @brief 计算参数矩阵
*/
void calibrate();

/**
* @brief 评估标定文件
* 
* @param image 当前帧
*/
void assess(cv::Mat& image);

/**
* @brief 评估标定文件（未使用）
*/
void CalibrationEvaluate();

/**
* @brief 计算坐标（未使用）
*/
void calRealPoint(std::vector<std::vector<cv::Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize);

    
}