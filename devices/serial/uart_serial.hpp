/**
 * @file uart_serial.hpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief  串口通讯
 * @date 2023-01-18
 * 
 * @copyright Copyright (c) 2023 Sirius
 * 
 */

#pragma once

#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <fmt/core.h>
#include <fmt/color.h>

#include <opencv2/opencv.hpp>

namespace uart {

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "uart_serial");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "uart_serial");

enum BufferLength {
  // 接收数据字节数
  REC_INFO_LENGTH   = 14,

  // 计算CRC校验码的发送数据的长度
  CRC_BUFF_LENGTH   = 13,

  // The send length of the array after append CRC auth code
  WRITE_BUFF_LENGTH = 15
};


// 我方颜色
enum Color {
  ALL,
  RED,
  BLUE
};

// 当前模式
enum RunMode {
  // 默认模式（基础自瞄）
  DEFAULT_MODE,
  // 基础自瞄模式
  AUTO_AIM,
  // 能量机关模式
  ENERGY_BUFF,
  // 击打哨兵模式
  SENTRY_STRIKE_MODE,
  // 反小陀螺模式
  TOP_MODE,
  // 内录模式
  RECORD_MODE,
  // 无人机模式
  PLANE_MODE,
  // 哨兵模式
  SENTINEL_AUTONOMOUS_MODE,
  // 雷达模式
  RADAR_MODE,
  // 相机标定模式
  CAMERA_CALIBRATION
};

// 机器人id
enum RobotID {
  HERO, // 英雄
  UAV, // 无人机
  ENGINEERING, // 工程
  INFANTRY, // 步兵
  SENTRY // 哨兵
};

// 串口信息
struct Serial_Config {
  std::string preferred_device        = "/dev/ttyACM0";
  int         set_baudrate            = 0;
  int         show_serial_information = 0;
};

// 接收的数据结构 头帧0x53 尾帧0x45
struct Receive_Data {
  int   my_color;  // 01 颜色
  int   now_run_mode; // 02 模式
  int   my_robot_id; // 03 机器人ID

  float yaw; // 0405 yaw轴
  float pitch; // 0607 pitch轴

  float yaw_velocity; // 0809 yaw轴加速度
  float pitch_velocity; // 1011 pitch轴加速度

  float bullet_velocity; // 12 子弹速度

  bool old_flag;

  union Bullet_Velocity_Info {
    short veloctiy; // 子弹速度（未解码）
    uint8_t arr_veloctiy = 0;
  } raw_bullet_velocity; // 子弹速度（未解码）

  union Yaw_Angle_Info {
    short yaw; // yaw轴（未解码）
    uint8_t arr_yaw[2] = {0};
  } raw_yaw_angle;  // yaw轴（未解码）

  union Yaw_Velocity_Info {
    short veloctiy; // yaw轴加速度（未解码）
    uint8_t arr_yaw_velocity[2] = {0};
  } raw_yaw_velocity; // yaw轴加速度（未解码）

  union Pitch_Angle_Info {
    short pitch; // pitch轴（未解码）
    uint8_t arr_pitch[2] = {0};
  } raw_pitch_angle; // pitch轴（未解码）

  union Pitch_Velocity_Info {
    short veloctiy; // pitch轴加速度（未解码）
    uint8_t arr_pitch_velocity[2] = {0};
  } raw_pitch_velocity; // pitch轴加速度（未解码）

  Receive_Data() {
    my_color                 = ALL;
    now_run_mode             = AUTO_AIM;
    my_robot_id              = INFANTRY;
    raw_bullet_velocity.veloctiy = 0;
    raw_yaw_angle.yaw            = 0;
    raw_yaw_velocity.veloctiy    = 0;
    raw_pitch_angle.pitch        = 0;
    raw_pitch_velocity.veloctiy  = 0;
    yaw = 0.f; // yaw轴
    pitch = 0.f; // pitch轴
    yaw_velocity = 0.f; // yaw轴加速度
    pitch_velocity = 0.f; // pitch轴加速度
    bullet_velocity = 0.f; // 子弹速度
    old_flag = false;
  }
};

// 发送的数据结构
struct Write_Data {
  int   data_type; // 01 识别信息
  int   is_shooting; // 02 射击信息 1发射 0不发射
  
  float yaw; // 0304 yaw轴数据

  float pitch; // 0506 pitch轴数据

  struct node {
    int x; // 0708 预测坐标x轴
    int y; // 0910 预测坐标y轴
  } cord;

  int  depth; // 1112 深度

  Write_Data() {
    data_type    = 0; // 01
    is_shooting  = 0; // 02
    yaw    = 0.f; // 0304
    pitch  = 0.f; // 0506
    cord.x       = 0; // 0708
    cord.y       = 0; // 0910
    depth        = 0; // 1112
  }
};

class SerialPort {
 public:
  SerialPort() = default;
  explicit SerialPort(std::string _serial_config);

  ~SerialPort();
  /**
   * @brief 返回接受数据的结构体
   * 
   * @return Receive_Data 
   */
  inline Receive_Data returnReceive() { return receive_data_; }
  /**
   * @brief 返回子弹速度
   * 
   * @return float 
   */
  inline float   returnReceiveBulletVelocity() { return receive_data_.bullet_velocity; }
  /**
   * @brief 返回机器人 ID
   * 
   * @return int 
   */
  inline int   returnReceiveRobotId()        { return receive_data_.my_robot_id; }
  /**
   * @brief 返回自身颜色
   * 
   * @return int 
   */
  inline int   returnReceiceColor()          { return receive_data_.my_color; }
  /**
   * @brief 返回模式选择
   * 
   * @return int 
   */
  inline int   returnReceiveMode()           { return receive_data_.now_run_mode; }
  /**
   * @brief 返回陀螺仪 Pitch 轴数据
   * 
   * @return float 
   */
  inline float returnReceivePitch()          { return receive_data_.pitch; }
  /**
   * @brief 返回陀螺仪 Yaw 轴数据
   * 
   * @return float 
   */
  inline float returnReceiveYaw()                  { return receive_data_.yaw; }
  /**
   * @brief 返回陀螺仪Yaw轴速度数据
   * 
   * @return float 
   */
  inline float returnReceiveYawVelocity()          { return receive_data_.yaw_velocity; }
  /**
   * @brief 返回陀螺仪Pitch轴速度数据
   * 
   * @return float 
   */
  inline float returnReceivePitchVelocity()        { return receive_data_.pitch_velocity;}

  inline bool returnOldFlag() {
    return receive_data_.old_flag;
  }

  /**
   * @brief 返回高八位数据
   *  @brief 发送数据
   * @param  _yaw             yaw 符号
   * @param  yaw              yaw 绝对值
   * @param  _pitch           pitch 符号
   * @param  pitch            pitch 绝对值
   * @param  depth            深度
   * @param  data_type        是否发现目标
   * @param  is_shooting      开火命令
   *
   * @param Byte 
   * @return unsigned char 
   */
  inline unsigned char returnHighBit(const int& Byte) {
    exchangebyte_ = (Byte >> 8) & 0xff;

    return exchangebyte_;
  }
  /**
   * @brief 返回低八位数据
   * 
   * @param Byte 
   * @return unsigned char 
   */
  inline unsigned char returnLowBit(const int& Byte) {
    exchangebyte_ = Byte & 0xff;

    return exchangebyte_;
  }
  /**
   * @brief 合并数据
   * 
   * @param highbit   高八位数据
   * @param lowbit    低八位数据
   * @return int16_t  合并后数据
   */
  inline int16_t mergeIntoBytes(const unsigned char& highbit,
                                const unsigned char& lowbit) {
    exchangebit_ = (highbit << 8) | lowbit;

    return exchangebit_;
  }

  /**
   * @brief 写入发送数据
   * @param data_type 是否发现目标
   * @param is_shooting 开火命令
   * @param yaw  yaw轴
   * @param pitch pitch轴
   * @param cord 预测坐标
   * @param depth 深度信息
   */
  void writeData(const int&     data_type,
                 const int&     is_shooting,
                 const int16_t& yaw,
                 const int16_t& pitch,
                 const Write_Data::node& cord,
                 const int16_t& depth);

  void writeData();
  /**
   * @brief 发送数据
   *
   * @param _write_data     需要发送的 Write_Data 结构体
   */
  void writeData(const Write_Data& _write_data);

  /**
   * @brief 发送数据
   *
   * @param _data_type    是否发现目标（装甲板数量）
   * @param _is_shooting  开火命令
   * @param _yaw          yaw 数据
   * @param _pitch        pitch 数据
   * @param _cord         预测坐标
   * @param _depth        深度
   */
  void updataWriteData(const int   _data_type, const int _is_shooting,
                       const float _yaw,   const float _pitch,
                       const Write_Data::node _cord, const int _depth);

  /**
   * @brief 数据转换为结构体
   *
   * @param _yaw          yaw 数据
   * @param _pitch        pitch 数据
   * @param _depth        深度
   * @param _data_type    是否发现目标
   * @param _is_shooting  开火命令
   * @return Write_Data   返回写入数据结构体
   */
  Write_Data gainWriteData(const int  _data_type, const int  _is_shooting,
                           const float _yaw,  const float _pitch,
                           const Write_Data::node _cord, const int  _depth);

  /**
   * @brief 接收数据
   */
  void receiveData();

  /**
   * @brief 接收数据是否正常
   * @return true  不正常
   * @return false 正常
   */
  bool isEmpty();
  
  /**
   * @brief 更新数据信息
   */
  void updateReceiveInformation();

 private:
  Serial_Config serial_config_;
  Receive_Data  receive_data_;
  Receive_Data  last_receive_data_;
  Write_Data    write_data_;

  int           fd;
  int           transform_arr_[4];
  unsigned char write_buff_[WRITE_BUFF_LENGTH];
  unsigned char crc_buff_[CRC_BUFF_LENGTH];
  unsigned char receive_buff_[REC_INFO_LENGTH];
  unsigned char receive_buff_temp_[REC_INFO_LENGTH * 2];
  unsigned char exchangebyte_;

  int16_t yaw_reduction_;
  int16_t pitch_reduction_;
  int16_t depth_reduction_;
  int16_t cord_reduction_x;
  int16_t cord_reduction_y;

  int16_t angle_reduction_;
  int16_t acceleration_reduction_;

  int16_t exchangebit_;

  ssize_t read_message_;
  ssize_t write_message_;

  bool get_flag;

  inline uint8_t checksumCRC(unsigned char* buf, uint16_t len);

  int pre_mode = 0;

/**
 * @brief Get the Data For CRC object
 * @param  data_type        是否发现目标
 * @param  is_shooting      开火命令
 * @param  _yaw             yaw 符号
 * @param  yaw              yaw 绝对值
 * @param  _pitch           pitch 符号
 * @param  pitch            pitch 绝对值
 * @param  depth            深度
 */
  void getDataForCRC(const int& data_type, const int& is_shooting,
                     const int16_t& yaw, const int16_t& pitch,
                     const Write_Data::node& cord, const int16_t& depth);

  /**
 * @brief 获取发送信息
 * @param data_type    是否发现目标
 * @param is_shooting  开火命令
 * @param yaw          yaw轴
 * @param pitch        pitch轴
 * @param cord         预测坐标
 * @param depth        深度
 * @param CRC          CRC 校验码
 */
  void getDataForSend(const int&     data_type, const int&     is_shooting,
                      const int16_t& yaw, const int16_t& pitch,
                      const Write_Data::node& cord, const int16_t& depth,
                      const uint8_t& CRC);
};

static constexpr unsigned char CRC8_Table[] = {
  0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,
  65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,
  130, 220, 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128,
  222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158,
  29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,
  102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25,  71,
  165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167,
  249, 27,  69,  198, 152, 122, 36,  248, 166, 68,  26,  153, 199, 37,  123,
  58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110, 237, 179, 81,
  15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,
  204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206,
  144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208,
  83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115, 202, 148, 118,
  40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,  9,
  235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,  233,
  183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
  116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107,
  53
};

}  // namespace uart
