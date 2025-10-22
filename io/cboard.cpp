#include "cboard.hpp"

#include "tools/crc.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0)
{
  auto com_port = read_yaml(config_path);

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[CBoard] Failed to open serial: {}", e.what());
    exit(1);
  }

  // 启动接收线程
  thread_ = std::thread(&CBoard::read_thread, this);

  // 阻塞等待第一个数据
  tools::logger()->info("[CBoard] Waiting for first quaternion...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[CBoard] Opened.");
}

CBoard::~CBoard()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  try {
    serial_.close();
  } catch (...) {
  }
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command) const
{
  tx_data_.control = command.control ? 1 : 0;
  tx_data_.shoot = command.shoot ? 1 : 0;
  tx_data_.yaw = command.yaw;
  tx_data_.pitch = command.pitch;
  tx_data_.horizon_distance = command.horizon_distance;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(const_cast<CBoard_TX_Data *>(&tx_data_)),
    sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(const_cast<CBoard_TX_Data *>(&tx_data_)),
                  sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[CBoard] Failed to write serial: {}", e.what());
  }
}

bool CBoard::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    return false;
  }
}

void CBoard::read_thread()
{
  tools::logger()->info("[CBoard] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    // 连接错误过多时尝试重连
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[CBoard] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 读取帧头 "SP"
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    // 检查帧头
    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    // 记录接收时间戳
    auto timestamp = std::chrono::steady_clock::now();

    // 读取剩余数据
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
              sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    // CRC16校验
    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[CBoard] CRC16 check failed.");
      continue;
    }

    error_count = 0;

    // 解析四元数 (wxyz顺序)
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({{q.normalized()}, timestamp});

    // 更新状态信息
    {
      std::lock_guard<std::mutex> lock(mutex_);

      bullet_speed = rx_data_.bullet_speed;
      ft_angle = rx_data_.ft_angle;

      // 工作模式转换
      switch (rx_data_.mode) {
        case 0:
          mode = Mode::idle;
          break;
        case 1:
          mode = Mode::auto_aim;
          break;
        case 2:
          mode = Mode::small_buff;
          break;
        case 3:
          mode = Mode::big_buff;
          break;
        case 4:
          mode = Mode::outpost;
          break;
        default:
          mode = Mode::idle;
          tools::logger()->warn("[CBoard] Invalid mode: {}", rx_data_.mode);
          break;
      }

      // 射击模式转换
      switch (rx_data_.shoot_mode) {
        case 0:
          shoot_mode = ShootMode::left_shoot;
          break;
        case 1:
          shoot_mode = ShootMode::right_shoot;
          break;
        case 2:
          shoot_mode = ShootMode::both_shoot;
          break;
        default:
          shoot_mode = ShootMode::left_shoot;
          tools::logger()->warn("[CBoard] Invalid shoot mode: {}", rx_data_.shoot_mode);
          break;
      }
    }

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }

  tools::logger()->info("[CBoard] read_thread stopped.");
}

void CBoard::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[CBoard] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[CBoard] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[CBoard] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  if (!yaml["com_port"]) {
    throw std::runtime_error("Missing 'com_port' in YAML configuration.");
  }

  return yaml["com_port"].as<std::string>();
}

}  // namespace io