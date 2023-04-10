/**
 * @file camera_calibration.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 相机标定
 * @date 2023-01-31
 * @copyright Copyright (c) 2023 Sirius
 */

#include "camera_calibration.hpp"
#define RELEASE

namespace cam{

  void create_images(cv::Mat& frame) {
#ifndef RELEASE
    imshow("frame", frame);
    ch = cv::waitKey(50);
#endif
   
    printf("%d ", ch);
    img=frame.clone();
    s = img.size();
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::drawChessboardCorners(img, cv::Size(7, 7), corners, ret);
    imagePoints.push_back(corners);
    objectPoints.push_back(obj);
    //cv::imshow("calibration-demo", img);
    if (ch == 'e') { // 获取图像
      cv::imwrite(fmt::format("{}/photos/{}.png",SAVE_FILE_PATH, std::to_string(index)) , frame);
      fmt::print("SUCCESS\n");
      index += 1;
    }
  }

  void calibrate(){
    cv::glob(fmt::format("{}/photos",SAVE_FILE_PATH), files);
    
    // 定义变量
    for (int i = 0; i < numCornersHor; i++){
      for (int j = 0; j < numCornersVer; j++){
        obj.push_back(cv::Point3f((float)j * numSquares, (float)i * numSquares, 0));
      }
    }

    // 发现棋盘格与绘制
    for (int i = 0; i < files.size(); i++) {
      fmt::print("image file:{}\n", files[i].c_str());
      img = cv::imread(files[i]);
      s = img.size();
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
#ifndef RELEASE
      cv::imshow("calibration-demo", gray);
#endif
      ret = cv::findChessboardCorners(gray, cv::Size(7, 7), corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
      if (ret) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        cv::drawChessboardCorners(img, cv::Size(7, 7), corners, ret);
        imagePoints.push_back(corners);
        objectPoints.push_back(obj);
#ifndef RELEASE
        cv::imshow("calibration-demo", img);
#endif
      }
    }
    // 相机校正
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
    calibrateCamera(objectPoints, imagePoints, s, intrinsic, distCoeffs, rvecs, tvecs);

    cv::FileStorage conf_file(fmt::format("{}/mv_camera_config_new.xml", SAVE_FILE_PATH), cv::FileStorage::WRITE);

    conf_file << "camera-matrix" << intrinsic << "distortion" << distCoeffs;

    conf_file.release();
  }

  void auto_create_images(cv::Mat& image){ 
    for (int i = 0; i < numCornersHor; i++){
      for (int j = 0; j < numCornersVer; j++){
        obj.push_back(cv::Point3f((float)j * numSquares, (float)i * numSquares, 0));
      }
    }
    s = image.size();
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
#ifndef RELEASE
    cv::imshow("calibration-demo", gray);
#endif
    ret = cv::findChessboardCorners(gray, cv::Size(7, 7), corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
    if (ret) {
      cv::imwrite(fmt::format("{}/photos/{}.png",SAVE_FILE_PATH, std::to_string(index)) , image);
      cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
      cv::drawChessboardCorners(image, cv::Size(7, 7), corners, ret);
      imagePoints.push_back(corners);
      objectPoints.push_back(obj);
#ifndef RELEASE
      cv::imshow("calibration-demo", image); // 显示识别到标定板的帧
#endif
      fmt::print("SUCCESS\n");
      index += 1;
    }

  }

  void assess(cv::Mat& image){
    cv::FileStorage camera_config(fmt::format("{}/mv_camera_config_337.xml", SAVE_FILE_PATH), cv::FileStorage::READ);
    camera_config["camera-matrix"] >> intrinsic;
    camera_config["distortion"] >> distCoeffs;

    cv::undistort(image, rectify, intrinsic, distCoeffs);
#ifndef RELEASE
    cv::imshow("ASSESS", rectify);
#endif
    camera_config.release();
  }

  void CalibrationEvaluate()//标定结束后进行评价  NOT FIXED
  {  
    std::vector<std::vector<cv::Point3f>> objRealPoint;
    double err=0;  
    double total_err=0;  
    //calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distCoeffs, rvecs, tvecs, 0);  
    cv::FileStorage camera_config(fmt::format("{}/mv_camera_config_new.xml", SAVE_FILE_PATH), cv::FileStorage::READ);
    camera_config["camera-matrix"] >> intrinsic;
    camera_config["distortion"] >> distCoeffs;

    cv::glob(fmt::format("{}/photos",SAVE_FILE_PATH), files);

    calRealPoint(objRealPoint, 7, 7, 7, 14);
    
    for (int i = 0; i < files.size(); i++) {
      fmt::print("image file:{}\n", files[i].c_str());
      img = cv::imread(files[i]);
      s = img.size();
      if(i == 0){
        cv::calibrateCamera(objRealPoint, corners, s, intrinsic, distCoeffs, rvecs, tvecs, 0);
      }
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      ret = cv::findChessboardCorners(gray, cv::Size(7, 7), corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
      if (ret) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        corners_.push_back(corners);
      }
    }
    
    fmt::print("每幅图像的定标误差:");  
    for (int i = 0; i < corners_.size(); i++)  
    {  
        std::vector<cv::Point2f> image_points2;  
        std::vector<cv::Point3f> tempPointSet = objRealPoint[i];  
        projectPoints(tempPointSet, rvecs[i], tvecs[i], intrinsic, distCoeffs, image_points2);  
        std::vector<cv::Point2f> tempImagePoint = corners_[i];  
        cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);  
        cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);  
        for (int j = 0; j < tempImagePoint.size(); j++)  
        {  
            image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);  
            tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);  
        }  
        err = cv::norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);  
        total_err = err + total_err;  
        fmt::print("第{}幅图像的平均误差:{}像素\n", std::to_string(i + 1), std::to_string(i + 1));
    } 
    fmt::print("总体平均误差:{}像素\n", std::to_string(total_err / (corners.size() + 1))); 

    camera_config.release();
  }  
  
  void calRealPoint(std::vector<std::vector<cv::Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
  {
    //  Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));    
    std::vector<cv::Point3f> imgpoint;  
    for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)  
    {  
        for (int colIndex = 0; colIndex < boardwidth; colIndex++)  
        {  
            //  imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);    
            imgpoint.push_back(cv::Point3f(colIndex * squaresize, rowIndex * squaresize, 0));  
        }  
    }  
    for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)  
    {  
        obj.push_back(imgpoint);  
    } 
  }
}
    



  











