/**
 * @file RobotVision_Sirius.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief 主函数
 * @date 2023-01-15
 * @copyright Copyright (c) 2023 Sirius
 */
#include "RobotVision_Sirius.hpp"
//#define VIDEO_DEBUG
#define MANUAL_FIRE
#define PARM_EDIT


int main() 
{
  // 调试信息
  fmt::print("[{}] RobotVision_Sirius built on g++ version: {}\n", idntifier, __VERSION__);
  fmt::print("[{}] RobotVision_Sirius config file path: {}\n", idntifier, CONFIG_FILE_PATH);
  
  cv::Mat src_img, roi_img; // src_img 读取图像 roi_img 截取图像

#ifndef VIDEO_DEBUG
  mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
  mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_1024, mindvision::EXPOSURE_20000));
  cv::VideoCapture cap_ = cv::VideoCapture(0);
#else
  
  cv::VideoCapture cap_(fmt::format("{}{}", SOURCE_PATH, "/video/2.mp4"));
  
#endif

  // 配置文件
  uart::SerialPort serial_ = uart::SerialPort(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/serial/uart_serial_config.xml"));
  
  basic_armor::Detector basic_armor_ = basic_armor::Detector(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/armor/basic_armor_config.xml"));

  basic_buff::Detector basic_buff_ = basic_buff::Detector(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/buff/basic_buff_config.xml"));

  basic_pnp::PnP pnp_ = basic_pnp::PnP(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/mv_camera_config_407.xml"), fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));

  onnx_inferring::model model_ = onnx_inferring::model(
    fmt::format("{}{}", SOURCE_PATH, "/module/ml/mnist-8.onnx"));

  angle_solve::solve solution;
  solution.set_config(fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/angle_solve_config.xml"));

  RecordMode::Record record_ = RecordMode::Record(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/record/recordpath_save.yaml"),
        fmt::format("{}{}", CONFIG_FILE_PATH, "/record/record_packeg/record.avi"),
        cv::Size(1280, 1024));  // 记得修改分辨率
  cv::VideoWriter vw_src;
  cv::FileStorage re_config_get(record_.video_save_path_, cv::FileStorage::READ);
  std::string save_path_ = "/record/video/";
  vw_src.open(CONFIG_FILE_PATH+save_path_+ to_string(time_now) + ".mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(1280, 1024), true);
  assert(vw_src.isOpened());
  basic_roi::RoI save_roi;
  fps::FPS       global_fps_;
  basic_roi::RoI roi_;
  
  while (true) {
    global_fps_.getTick();
#ifndef VIDEO_DEBUG
    if (mv_capture_->isindustryimgInput()) {
      src_img = mv_capture_->image();
    } else {
    cap_.read(src_img);
    }
#else
      cap_.read(src_img);
      cv::waitKey(30);
#endif
    if (!src_img.empty()) {
      cv::line(src_img,(cv::Point){640,0},(cv::Point){640,1024},cv::Scalar(255,0,0));
      cv::line(src_img,(cv::Point){0,512},(cv::Point){1280,512},cv::Scalar(255,0,0));
      fire = false;
      serial_.updateReceiveInformation();
      
      /*if(serial_.returnOldFlag()) {
        continue;
      }*/
      switch (serial_.returnReceiveMode()) {
      // 基础自瞄模式
      case uart::AUTO_AIM:
       if (basic_armor_.runBasicArmor(src_img, serial_.returnReceive())) {
          solution.angleSolve(basic_armor_.returnFinalArmorRotatedRect(0), src_img.size().height, src_img.size().width, serial_);
          //
          //pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
        }
#ifdef MANUAL_FIRE
        if(basic_armor_.returnArmorNum() != 0) {
          if(cv::waitKey(1) == 'f') {
            fire = true;
          } else {
            fire = false;
          }
        }
#endif
        /*serial_.updataWriteData(basic_armor_.returnArmorNum(), fire,
                                pnp_.returnYawAngle(),
                                pnp_.returnPitchAngle(),
                                basic_armor_.returnArmorCenter(0),
                                pnp_.returnDepth());*/
        serial_.updataWriteData(basic_armor_.returnArmorNum(), fire,
                                solution.returnYawAngle(),
                                solution.returnPitchAngle(),
                                basic_armor_.returnArmorCenter(0),
                                0);
        break;
      // 能量机关
      case uart::ENERGY_BUFF:
        serial_.writeData(basic_buff_.runTask(src_img, serial_.returnReceive()));
        break;
      // 击打哨兵模式
      case uart::SENTRY_STRIKE_MODE:
        if (basic_armor_.sentryMode(src_img, serial_.returnReceive())) {
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                        basic_armor_.returnFinalArmorDistinguish(0),
                        basic_armor_.returnFinalArmorRotatedRect(0));
          serial_.updataWriteData(basic_armor_.returnArmorNum(), 0,
                                  pnp_.returnYawAngle(),
                                  pnp_.returnPitchAngle(),
                                  basic_armor_.returnArmorCenter(0),
                                  pnp_.returnDepth());
        } else {
          serial_.updataWriteData(basic_armor_.returnLostCnt() > 0 ? 1 : 0, 0,
                                  -pnp_.returnYawAngle(),
                                  pnp_.returnPitchAngle(),
                                  basic_armor_.returnArmorCenter(0),
                                  pnp_.returnDepth());
        }

        break;
      // 反小陀螺模式（暂未完善）
      case uart::TOP_MODE:
        roi_img = roi_.returnROIResultMat(src_img);
        if (basic_armor_.runBasicArmor(roi_img, serial_.returnReceive())) {
          basic_armor_.fixFinalArmorCenter(0, roi_.returnRectTl());
          roi_.setLastRoiRect(basic_armor_.returnFinalArmorRotatedRect(0),
                              basic_armor_.returnFinalArmorDistinguish(0));
          pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                        basic_armor_.returnFinalArmorDistinguish(0),
                        basic_armor_.returnFinalArmorRotatedRect(0));
          serial_.updataWriteData(basic_armor_.returnArmorNum(), 0,
                                  pnp_.returnYawAngle(),
                                  pnp_.returnPitchAngle(),
                                  basic_armor_.returnArmorCenter(0),
                                  pnp_.returnDepth());
        } else {
          serial_.updataWriteData(basic_armor_.returnLostCnt() > 0 ? 1 : 0, 0,
                                  -pnp_.returnYawAngle(),
                                  pnp_.returnPitchAngle(),
                                  basic_armor_.returnArmorCenter(0),
                                  pnp_.returnDepth());
        }
        roi_.setLastRoiSuccess(basic_armor_.returnArmorNum());

        break;
      // 录制视频
      case uart::RECORD_MODE:
      vw_src << src_img;
        break;
      // 无人机模式（空缺）
      case uart::PLANE_MODE:
        break;
      // 哨兵模式（添加数字识别便于区分工程和其他车辆）
      case uart::SENTINEL_AUTONOMOUS_MODE:
        if (basic_armor_.runBasicArmor(src_img, serial_.returnReceive())) {
          if (basic_armor_.returnFinalArmorDistinguish(0) == 1) {
            pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
              basic_armor_.returnFinalArmorDistinguish(0),
              basic_armor_.returnFinalArmorRotatedRect(0));
            serial_.updataWriteData(basic_armor_.returnArmorNum(), 0,
                                    pnp_.returnYawAngle(),
                                    pnp_.returnPitchAngle(),
                                    basic_armor_.returnArmorCenter(0),
                                    pnp_.returnDepth());
          } else {
            for (int i = 0; i < basic_armor_.returnArmorNum(); i++) {
              if (model_.inferring(save_roi.cutRoIRotatedRect(src_img,
                basic_armor_.returnFinalArmorRotatedRect(i)), 0, 0, src_img) == 2) {
                if (basic_armor_.returnArmorNum() > 1) {
                  pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                                basic_armor_.returnFinalArmorDistinguish(i + 1),
                                basic_armor_.returnFinalArmorRotatedRect(i + 1));
                  serial_.updataWriteData(basic_armor_.returnArmorNum(), 0,
                                          pnp_.returnYawAngle(),
                                          pnp_.returnPitchAngle(),
                                          basic_armor_.returnArmorCenter(0),
                                          pnp_.returnDepth());
                  break;
                } else {
                  serial_.updataWriteData(0, 0,
                                          pnp_.returnYawAngle(),
                                          pnp_.returnPitchAngle(),
                                          basic_armor_.returnArmorCenter(0),
                                          pnp_.returnDepth());
                  break;
                }
              } else {
                pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                              basic_armor_.returnFinalArmorDistinguish(0),
                              basic_armor_.returnFinalArmorRotatedRect(0));
                serial_.updataWriteData(basic_armor_.returnArmorNum(), 0,
                                        pnp_.returnYawAngle(),
                                        pnp_.returnPitchAngle(),
                                        basic_armor_.returnArmorCenter(0),
                                        pnp_.returnDepth());
                break;
              }
            }
          }
        }
        break;
      // 雷达模式
      case uart::RADAR_MODE:
        break;
      // 相机标定
      case uart::CAMERA_CALIBRATION:
        //cam::create_images(src_img);
        //cam::calibrate();
        //cam::auto_create_images(src_img);
        cam::assess(src_img);
        //cam::CalibrationEvaluate();
        break;
      default: // 默认进入基础自瞄 
        if (basic_armor_.runBasicArmor(src_img, serial_.returnReceive())) {
            pnp_.solvePnP(serial_.returnReceiveBulletVelocity(),
                          basic_armor_.returnFinalArmorDistinguish(0),
                          basic_armor_.returnFinalArmorRotatedRect(0));
        }
        serial_.updataWriteData(basic_armor_.returnArmorNum(), 0,
                                pnp_.returnYawAngle(),
                                pnp_.returnPitchAngle(),
                                basic_armor_.returnArmorCenter(0),
                                pnp_.returnDepth());
        break;
      }
    }
    else{
#ifdef VIDEO_DEBUG
      //cap_.open(fmt::format("{}{}", SOURCE_PATH, "/video/1080.mp4"));
#endif
    }
    if (record_.last_mode_ != uart::RECORD_MODE && serial_.returnReceiveMode() == uart::RECORD_MODE) {
      vw_src.release();
    }
    record_.last_mode_ = serial_.returnReceiveMode();
    // 非击打哨兵模式时初始化
    if (serial_.returnReceiveMode() != uart::SENTRY_STRIKE_MODE) {
      basic_armor_.initializationSentryMode();
    }
#ifndef VIDEO_DEBUG
    mv_capture_->cameraReleasebuff();
#endif
    basic_armor_.freeMemory(fmt::format("{}{}", CONFIG_FILE_PATH, "/armor/basic_armor_config_new.xml"));

#ifndef RELEASE
    cv::imshow("dafule", src_img);
    vw_src << src_img;
    if (cv::waitKey(1) == 'q') {
      vw_src.release();
      return 0;
    }
#else
    usleep(1);
#endif
    // 看门狗放置相机掉线
    global_fps_.calculateFPSGlobal();
    if (global_fps_.returnFps() > 500) {
#ifndef VIDEO_DEBUG
      mv_capture_->~VideoCapture();
#endif
      static int counter_for_dev {100};
      static int counter_for_new {30};
      while (!utils::resetMVCamera()) {
        if (!--counter_for_dev) {
          //int i [[maybe_unused]] = std::system("echo 1 | sudo -S reboot");
        }
        usleep(100);
      }
      usleep(100);
#ifndef VIDEO_DEBUG
      mv_capture_ = new mindvision::VideoCapture(mindvision::CameraParam(
          0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_20000));
#endif
      if (!--counter_for_new) {
        //int i [[maybe_unused]] = std::system("echo 1 | sudo -S reboot");
      }
    }
    
  }
  return 0;
}
