/**
 * @file uart_serial.cpp
 * @author dgsyrc (yrcminecraft@foxmail.com)
 * @brief  串口通讯
 * @date 2023-01-18
 * @copyright Copyright (c) 2023 Sirius
 * 
 */

#include "uart_serial.hpp"

namespace uart {

SerialPort::SerialPort(std::string _serial_config) {
  cv::FileStorage fs_serial(_serial_config, cv::FileStorage::READ);

  fs_serial["PREFERRED_DEVICE"]        >> serial_config_.preferred_device;
  fs_serial["SET_BAUDRATE"]            >> serial_config_.set_baudrate;
  fs_serial["SHOW_SERIAL_INFORMATION"] >> serial_config_.show_serial_information;

  const char* DeviceName[] = {serial_config_.preferred_device.c_str(), "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};

  struct termios newstate;
  bzero(&newstate, sizeof(newstate));

  for (size_t i = 0; i != sizeof(DeviceName) / sizeof(char*); ++i) {
    fd = open(DeviceName[i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
      fmt::print("[{}] Open serial device failed: {}\n", idntifier_red, DeviceName[i]);
    } else {
      fmt::print("[{}] Open serial device success: {}\n", idntifier_green, DeviceName[i]);

      break;
    }
  }

  switch (serial_config_.set_baudrate) {
    case 1:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
    case 10:
      cfsetospeed(&newstate, B921600);
      cfsetispeed(&newstate, B921600);
      break;
    default:
      cfsetospeed(&newstate, B115200);
      cfsetispeed(&newstate, B115200);
      break;
  }

  newstate.c_cflag |= CLOCAL | CREAD;
  newstate.c_cflag &= ~CSIZE;
  newstate.c_cflag &= ~CSTOPB;
  newstate.c_cflag |= CS8;
  newstate.c_cflag &= ~PARENB;

  newstate.c_cc[VTIME] = 0;
  newstate.c_cc[VMIN]  = 0;

  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &newstate);
}

SerialPort::~SerialPort(void) {
  if (!close(fd)) { fmt::print("[{}] Close serial device success: {}\n", idntifier_green, fd); }
}

/* 接收的数据:
 *  0:      'S'
 *  1:      color
 *  2:      model
 *  3:      robot_id
 *  4~7:    yaw_angle (union)
 *  8~11:   pitch_angle (union)
 *  12~13:  yaw_velocity
 *  14~15:  pitch_velocity
 *  16:     bullet_velocity
 *  17:     'E'
 */
void SerialPort::receiveData() {
  // 初始化接收数据为0
  memset(receive_buff_, 0, REC_INFO_LENGTH * 2);
  // 接收数据
  read_message_ = read(fd, receive_buff_temp_, sizeof(receive_buff_temp_));
  for (size_t i = 0; i != sizeof(receive_buff_temp_); ++i) {
    if (receive_buff_temp_[i] == 'S' && receive_buff_temp_[i + sizeof(receive_buff_) - 1] == 'E') {
      if (serial_config_.show_serial_information == 1) {
        fmt::print("[{}] receiveData() ->", idntifier_green);
        for (size_t j = 0; j != sizeof(receive_buff_); ++j) {
          receive_buff_[j] = receive_buff_temp_[i + j];
          fmt::print(" {:d}", receive_buff_[j]);
        }
        fmt::print("\n");
      } else {
        for (size_t j = 0; j != sizeof(receive_buff_); ++j) {
          receive_buff_[j] = receive_buff_temp_[i + j];
        }
      }

      break;
    }
    
  }
  tcflush(fd, TCIFLUSH);
}

/**
 * @brief 写入发送数据
 * 
 * @param _yaw yaw轴符号
 * @param yaw  yaw轴
 * @param _pitch pitch轴符号
 * @param pitch pitch轴
 * @param depth 深度信息
 * @param 
 * @return void 
 */
void SerialPort::writeData(const int&     _yaw,
                           const int16_t& yaw,
                           const int&     _pitch,
                           const int16_t& pitch,
                           const int16_t& depth,
                           const int16_t& predict_cord,
                           const int16_t&     predict_length,
                           const int16_t&     predict_width,
                           const int&     data_type,
                           const int&     is_shooting) {
  getDataForCRC(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth, predict_cord, predict_length, predict_width);

  uint8_t CRC = checksumCRC(crc_buff_, sizeof(crc_buff_));

  getDataForSend(data_type, is_shooting, _yaw, yaw, _pitch, pitch, depth, CRC, predict_cord, predict_length, predict_width);

  //memset(write_buff_, 0, sizeof(write_buff_));
  write_buff_[0]='S';
  write_buff_[18]='E';
  write_message_ = write(fd, write_buff_, sizeof(write_buff_));

  if (serial_config_.show_serial_information == 1) {
    yaw_reduction_   = mergeIntoBytes(write_buff_[5],  write_buff_[4]);
    pitch_reduction_ = mergeIntoBytes(write_buff_[8],  write_buff_[7]);
    depth_reduction_ = mergeIntoBytes(write_buff_[10], write_buff_[9]);
    predict_cord_reduction_ = mergeIntoBytes(write_buff_[12], write_buff_[13]);

    //fmt::print("[{}] writeData() ->", idntifier_green);
    /*for (size_t i = 0; i < 4; ++i) { fmt::print(" {}", write_buff_[i]); }
      fmt::print(" {} {} {} {}",
      static_cast<float>(yaw_reduction_) / 100,
      static_cast<int>(write_buff_[6]),
      static_cast<float>(pitch_reduction_) / 100,
      static_cast<float>(depth_reduction_));
    for (size_t i = 11; i < 19; ++i) { fmt::print(" {}", write_buff_[i]); }*/
    //fmt::print("\n");

    yaw_reduction_   = 0x0000;
    pitch_reduction_ = 0x0000;
    depth_reduction_ = 0x0000;
  }
}

void SerialPort::writeData(const Write_Data& _write_data) {
  write_data_.data_type    = _write_data.data_type > 1 ? 1 : _write_data.data_type;
  write_data_.is_shooting  = _write_data.is_shooting;
  write_data_.symbol_yaw   = _write_data.yaw_angle >= 0 ? 1 : 0;
  write_data_.yaw_angle    = fabs(_write_data.yaw_angle) * 100;
  write_data_.symbol_pitch = _write_data.pitch_angle >= 0 ? 1 : 0;
  write_data_.pitch_angle  = fabs(_write_data.pitch_angle) * 100;
  write_data_.depth        = _write_data.depth;
  write_data_.predict_cord = 0;
  write_data_.predict_length = 0;
  write_data_.predict_width = 0;

  writeData(write_data_.symbol_yaw,
            write_data_.yaw_angle,
            write_data_.symbol_pitch,
            write_data_.pitch_angle,
            write_data_.depth,
            write_data_.data_type,
            write_data_.is_shooting,
            write_data_.predict_cord,
            write_data_.predict_length,
            write_data_.predict_width);
}

void SerialPort::writeData() {
  writeData(write_data_.symbol_yaw,
            write_data_.yaw_angle,
            write_data_.symbol_pitch,
            write_data_.pitch_angle,
            write_data_.depth,
            write_data_.data_type,
            write_data_.is_shooting,
            write_data_.predict_cord,
            write_data_.predict_length,
            write_data_.predict_width);
}

void SerialPort::updataWriteData(const float _yaw,
                                 const float _pitch,
                                 const int   _depth,
                                 const int   _predict_cord,
                                 const int   _predict_length,
                                 const int   _predict_width,
                                 const int   _data_type,
                                 const int   _is_shooting) {
  write_data_.data_type    = _data_type > 1 ? 1 : _data_type;
  write_data_.is_shooting  = _is_shooting;
  write_data_.symbol_yaw   = _yaw >= 0 ? 1 : 0;
  write_data_.yaw_angle    = fabs(_yaw) * 100;
  write_data_.symbol_pitch = _pitch >= 0 ? 1 : 0;
  write_data_.pitch_angle  = fabs(_pitch) * 100;
  write_data_.depth        = _depth;
  write_data_.predict_cord = 0;
  write_data_.predict_length = 0;
  write_data_.predict_width = 0;
  writeData();
}

Write_Data SerialPort::gainWriteData(const float _yaw,
                                     const float _pitch,
                                     const int   _depth,
                                     const int   _predict_cord,
                                     const int   _predict_length,
                                     const int   _predict_width,
                                     const int   _data_type,
                                     const int   _is_shooting) {
  Write_Data write_data;

  write_data.data_type    = _data_type > 1 ? 1 : _data_type;
  write_data.is_shooting  = _is_shooting;
  write_data.symbol_yaw   = _yaw >= 0 ? 1 : 0;
  write_data.yaw_angle    = fabs(_yaw) * 100;
  write_data.symbol_pitch = _pitch >= 0 ? 1 : 0;
  write_data.pitch_angle  = fabs(_pitch) * 100;
  write_data.depth        = _depth;
  write_data_.predict_cord = 0;
  write_data_.predict_length = 0;
  write_data_.predict_width = 0;
  return write_data;
}

uint8_t SerialPort::checksumCRC(unsigned char* buf, uint16_t len) {
  uint8_t check = 0;

  while (len--) { check = CRC8_Table[check ^ (*buf++)]; }

  return check;
}

void SerialPort::getDataForCRC(const int&     data_type,
                               const int&     is_shooting,
                               const int&     _yaw,
                               const int16_t& yaw,
                               const int&     _pitch,
                               const int16_t& pitch,
                               const int16_t& depth,
                               const int16_t& predict_cord,
                               const int16_t& predict_length,
                               const int16_t& predict_width) {
  crc_buff_[0]  = 0x53;
  crc_buff_[1]  = static_cast<unsigned char>(data_type);
  crc_buff_[2]  = static_cast<unsigned char>(is_shooting);
  crc_buff_[3]  = static_cast<unsigned char>(_yaw);
  crc_buff_[4]  = returnLowBit(yaw);
  crc_buff_[5]  = returnHighBit(yaw);
  crc_buff_[6]  = static_cast<unsigned char>(_pitch);
  crc_buff_[7]  = returnLowBit(pitch);
  crc_buff_[8]  = returnHighBit(pitch);
  crc_buff_[9]  = returnLowBit(depth);
  crc_buff_[10] = returnHighBit(depth);
}

void SerialPort::getDataForSend(const int&     data_type,
                                const int&     is_shooting,
                                const int&     _yaw,
                                const int16_t& yaw,
                                const int&     _pitch,
                                const int16_t& pitch,
                                const int16_t& depth,
                                const uint8_t& CRC,
                                const int16_t& predict_cord,
                                const int16_t& predict_length,
                                const int16_t& predict_width) {
  write_buff_[0]  = 0x53;
  write_buff_[1]  = static_cast<unsigned char>(data_type);
  write_buff_[2]  = static_cast<unsigned char>(is_shooting);
  write_buff_[3]  = static_cast<unsigned char>(_yaw);
  write_buff_[4]  = returnLowBit(yaw);
  write_buff_[5]  = returnHighBit(yaw);
  write_buff_[6]  = static_cast<unsigned char>(_pitch);
  write_buff_[7]  = returnLowBit(pitch);
  write_buff_[8]  = returnHighBit(pitch);
  write_buff_[9]  = returnLowBit(depth);
  write_buff_[10] = returnHighBit(depth);
  write_buff_[11] = CRC & 0xff;
  write_buff_[12]  = returnLowBit(predict_cord);
  write_buff_[13] = returnHighBit(predict_cord);
  write_buff_[14] = returnLowBit(predict_length);
  write_buff_[15] = returnHighBit(predict_length);
  write_buff_[16] = returnLowBit(predict_width);
  write_buff_[17] = returnHighBit(predict_width);
  write_buff_[18] = 0x45;
}

bool SerialPort::isEmpty() {
  if (receive_buff_[0] != '0' || receive_buff_[REC_INFO_LENGTH - 1] != '0') {
    return false;
  } else {
    return true;
  }
}

void SerialPort::updateReceiveInformation() {
  receiveData();

  if (isEmpty()) {
    return;
  } else {
    last_receive_data_ = receive_data_;
  }


  // BUG？
  /*for (size_t i = 0; i != sizeof(transform_arr_) / sizeof(transform_arr_[0]); ++i) {
    transform_arr_[i] = receive_buff_[i + 1] - '0';
    fmt::print("[info] transform_arr_[{:d}]:{:d}", i, transform_arr_[i]);
  }*/
  //switch (transform_arr_[0]) {
  switch (receive_buff_[1]) {
    case RED:
      receive_data_.my_color = RED;
      fmt::print("[info] My color is RED\n");
      break;
    case BLUE:
      receive_data_.my_color = BLUE;
      fmt::print("[info] My color is BLUE\n");
      break;
    default:
      receive_data_.my_color = ALL;
      fmt::print("[info] My color is ALL\n");
      break;
  }
  //switch (transform_arr_[1]) {
  switch (receive_buff_[2]) {
  case SUP_SHOOT:
    receive_data_.now_run_mode = SUP_SHOOT;
    break;
  case ENERGY_AGENCY:
    receive_data_.now_run_mode = ENERGY_AGENCY;
    break;
  case SENTRY_STRIKE_MODE:
    receive_data_.now_run_mode = SENTRY_STRIKE_MODE;
    break;
  case TOP_MODE:
    receive_data_.now_run_mode = TOP_MODE;
    break;
  case RECORD_MODE:
    receive_data_.now_run_mode = RECORD_MODE;
    break;
  case PLANE_MODE:
    receive_data_.now_run_mode = PLANE_MODE;
    break;
  case SENTINEL_AUTONOMOUS_MODE:
    receive_data_.now_run_mode = SENTINEL_AUTONOMOUS_MODE;
    break;
  case RADAR_MODE:
    receive_data_.now_run_mode = RADAR_MODE;
  default:
    receive_data_.now_run_mode = SUP_SHOOT;
    break;
  }

  //switch (transform_arr_[2]) {
  switch (receive_buff_[3]) {
    case HERO:
      receive_data_.my_robot_id = HERO;
      break;
    case ENGINEERING:
      receive_data_.my_robot_id = ENGINEERING;
      break;
    case INFANTRY:
      receive_data_.my_robot_id = INFANTRY;
      break;
    case UAV:
      receive_data_.my_robot_id = UAV;
      break;
    case SENTRY:
      receive_data_.my_robot_id = SENTRY;
      break;
    default:
      receive_data_.my_robot_id = INFANTRY;
      break;
  }

  /*receive_data_.bullet_velocity = receive_buff_[14]; // -2 ?
  fmt::print("[info] bullet_velocity:{:d}\n", receive_data_.bullet_velocity);*/

  for (size_t i = 0; i != sizeof(receive_data_.raw_yaw_angle.arr_yaw); ++i) {
    receive_data_.raw_yaw_angle.arr_yaw[i] = receive_buff_[i + 4];
  }

  for (size_t i = 0; i != sizeof(receive_data_.raw_pitch_angle.pitch); ++i) {
    receive_data_.raw_pitch_angle.arr_pitch[i] = receive_buff_[i + 6];
  }

  for (size_t i = 0; i != sizeof(receive_data_.raw_pitch_velocity.veloctiy); ++i) {
    receive_data_.raw_pitch_velocity.arr_pitch_velocity[i] = receive_buff_[i + 8];
  }

  for (size_t i = 0; i != sizeof(receive_data_.raw_yaw_velocity.veloctiy); ++i) {
    receive_data_.raw_yaw_velocity.arr_yaw_velocity[i] = receive_buff_[i + 10];
  }

  receive_data_.raw_bullet_velocity.arr_veloctiy = receive_buff_[12];
}
}  // namespace uart
