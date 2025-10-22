#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
// 接收数据结构体 - 从C板接收的数据
struct __attribute__((packed)) CBoard_RX_Data
{
  uint8_t head[2] = {'S', 'P'};
  float q[4];                // wxyz顺序的四元数
  double bullet_speed;       // 子弹速度
  uint8_t mode;              // 0: idle, 1: auto_aim, 2: small_buff, 3: big_buff, 4: outpost
  uint8_t shoot_mode;        // 0: left_shoot, 1: right_shoot, 2: both_shoot (哨兵专用)
  float ft_angle;            // FT角度 (无人机专用)
  uint16_t crc16;
};

// 发送数据结构体 - 向C板发送的数据
struct __attribute__((packed)) CBoard_TX_Data
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t control;           // 控制标志: 0或1
  uint8_t shoot;             // 射击标志: 0或1
  float yaw;                 // Yaw角度 (单位: rad)
  float pitch;               // Pitch角度 (单位: rad)
  float horizon_distance;    // 水平距离 (单位: m)
  uint16_t crc16;
};

enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  //无人机专有

  CBoard(const std::string & config_path);

  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command) const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  CBoard_RX_Data rx_data_;
  CBoard_TX_Data tx_data_;

  tools::ThreadSafeQueue<IMUData> queue_{5000};
  IMUData data_ahead_;
  IMUData data_behind_;

  void read_thread();
  bool read(uint8_t * buffer, size_t size);
  void reconnect();

  std::string read_yaml(const std::string & config_path);
};

}  // namespace io

#endif  // IO__CBOARD_HPP