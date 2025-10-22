# CBoard 串口通信改造指南

## 概述

CBoard 模块已从 CAN 通信改为串口通信，实现与 Gimbal 模块通信协议的统一。本文档详细说明改造内容、使用方法和迁移指南。

## 主要改进

### 1. 通信方式改变

| 方面 | 旧版本 (CAN) | 新版本 (串口) |
|------|-------------|------------|
| **通信协议** | SocketCAN + CAN帧 | Serial + 结构化数据包 |
| **接收方式** | 回调函数处理 | 独立线程处理 |
| **数据验证** | 无 | CRC16校验 |
| **自动重连** | 无 | 有（5000错误后触发） |
| **与Gimbal兼容** | ✗ | ✓ |

### 2. 数据结构变化

#### 接收数据结构体 (CBoard_RX_Data)

```cpp
struct __attribute__((packed)) CBoard_RX_Data {
  uint8_t head[2] = {'S', 'P'};    // 帧头
  float q[4];                       // 四元数 (w, x, y, z)
  double bullet_speed;              // 子弹速度 (m/s)
  uint8_t mode;                     // 工作模式 (0-4)
  uint8_t shoot_mode;               // 射击模式 (0-2)
  float ft_angle;                   // FT角度 (rad)
  uint16_t crc16;                   // CRC16校验
};
```

**工作模式对应表**
- 0: idle - 空闲
- 1: auto_aim - 自瞄
- 2: small_buff - 小符
- 3: big_buff - 大符
- 4: outpost - 前哨站

**射击模式对应表**
- 0: left_shoot - 左侧射击（哨兵）
- 1: right_shoot - 右侧射击（哨兵）
- 2: both_shoot - 双侧射击（哨兵）

#### 发送数据结构体 (CBoard_TX_Data)

```cpp
struct __attribute__((packed)) CBoard_TX_Data {
  uint8_t head[2] = {'S', 'P'};    // 帧头
  uint8_t control;                  // 控制标志 (0/1)
  uint8_t shoot;                    // 射击标志 (0/1)
  float yaw;                        // Yaw角度 (rad)
  float pitch;                      // Pitch角度 (rad)
  float horizon_distance;           // 水平距离 (m)
  uint16_t crc16;                   // CRC16校验
};
```

## 配置文件迁移

### 旧版本配置 (CAN)

```yaml
# configs/infantry3.yaml (CAN方案)
can_interface: "can0"
quaternion_canid: 0x200
bullet_speed_canid: 0x201
send_canid: 0x202
```

### 新版本配置 (串口)

```yaml
# configs/infantry3.yaml (串口方案)
com_port: "/dev/gimbal"
```

**com_port 说明**
- `/dev/gimbal`: 通过 udev 规则固定的串口符号链接（推荐）
- `/dev/ttyACM0`: USB虚拟串口设备（可能变化）
- `/dev/ttyUSB0`: USB串转串设备
- 需要在系统中通过 udev 规则提前配置好符号链接

## 代码变化

### 头文件改变 (cboard.hpp)

**删除项**
```cpp
#include "io/socketcan.hpp"                    // 删除CAN库
#include <functional>                          // 删除回调相关

SocketCAN can_;                                // 删除CAN对象
std::function<...> callback;                  // 删除回调
int quaternion_canid_, bullet_speed_canid_, send_canid_; // 删除CAN ID配置
```

**新增项**
```cpp
#include "serial/serial.h"                     // 新增串口库
#include <atomic>                              // 线程控制
#include <mutex>                               // 互斥锁
#include <thread>                              // 接收线程

serial::Serial serial_;                        // 串口对象
std::thread thread_;                           // 接收线程
std::atomic<bool> quit_;                       // 退出信号
std::mutex mutex_;                             // 线程安全

struct CBoard_RX_Data rx_data_;                // 接收数据缓冲
struct CBoard_TX_Data tx_data_;                // 发送数据缓冲

void read_thread();                            // 接收线程入口
bool read(uint8_t* buffer, size_t size);      // 串口读操作
void reconnect();                              // 重连逻辑
```

### 实现改变 (cboard.cpp)

**构造函数**
- 旧: 初始化CAN, 通过回调接收数据
- 新: 初始化串口，启动接收线程

**send() 函数**
- 旧: 构造CAN帧，通过CAN总线发送
- 新: 填充结构体，计算CRC16，通过串口发送

**新增: read_thread()**
- 独立线程持续读取串口数据
- 验证帧头、CRC16
- 更新IMU队列和状态变量

**新增: reconnect()**
- 连接失败时自动重连
- 最多尝试10次，每次间隔1秒

## 使用示例

### 基本使用（无变化）

```cpp
#include "io/cboard.hpp"

int main() {
  // 初始化 - 自动打开串口、启动接收线程
  io::CBoard board("configs/infantry3.yaml");
  
  // 查询某时刻的姿态（支持插值）
  auto q = board.imu_at(std::chrono::steady_clock::now());
  
  // 读取状态信息
  std::cout << "Bullet speed: " << board.bullet_speed << " m/s\n";
  std::cout << "Mode: " << io::MODES[board.mode] << "\n";
  
  // 发送控制指令
  io::Command cmd;
  cmd.control = true;
  cmd.shoot = false;
  cmd.yaw = 0.1;
  cmd.pitch = -0.05;
  board.send(cmd);
  
  return 0;
}
```

## 性能对比

### 吞吐量
- CAN方案: ~1000 Hz (CAN总线速率1 Mbps)
- 串口方案: ~100 Hz (串口速率115200 bps) 或更高

### 延迟
- CAN方案: 1-2 ms (硬件中断)
- 串口方案: 5-10 ms (软件轮询)

### 可靠性
- CAN方案: 无校验
- 串口方案: CRC16校验 + 自动重连

## 调试建议

### 1. 验证串口连接

```bash
# 查看串口设备
ls -l /dev/gimbal

# 使用screen测试通信
sudo apt install screen
screen /dev/gimbal 115200
```

### 2. 查看日志

```bash
# CBoard会输出调试日志
./build/your_program 2>&1 | grep "CBoard"
```

### 3. 常见问题

**问题1: 打不开串口**
```
[CBoard] Failed to open serial: No such file or directory
```
解决: 检查`/dev/gimbal`是否存在，是否有读写权限

**问题2: CRC校验失败频繁**
```
[CBoard] CRC16 check failed.
```
解决: 检查串口波特率配置，检查下位机是否正确发送数据

**问题3: 自动重连频繁**
```
[CBoard] Too many errors, attempting to reconnect...
```
解决: 检查串口接线，检查USB驱动，尝试重新插拔

## 迁移清单

- [ ] 更新配置文件 (YAML) 中的com_port参数
- [ ] 检查CMakeLists.txt是否链接serial库
- [ ] 重新编译项目
- [ ] 使用screen/minicom验证串口通信
- [ ] 运行cboard_test程序测试
- [ ] 在实车上测试各模式切换
- [ ] 验证IMU数据准确性
- [ ] 验证控制指令正确发送

## 向后兼容性

**重要**: 此改动**不向后兼容**CAN配置。需要：
1. 硬件上: 将下位机连接改为USB虚拟串口（MicroUSB）
2. 软件上: 更新配置文件为新格式
3. 代码上: 无需修改使用CBoard的业务代码

## 参考资源

- `io/cboard.hpp` - 头文件定义
- `io/cboard.cpp` - 实现文件
- `io/gimbal/gimbal.hpp/cpp` - Gimbal参考实现
- `tools/crc.hpp` - CRC校验工具
- `tests/cboard_test.cpp` - 单元测试

