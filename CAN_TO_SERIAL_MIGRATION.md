# CBoard 通信方式迁移文档：CAN → 虚拟串口

**迁移日期**：2025年10月23日  
**方案选择**：方案一（完全替换）  
**状态**：✅ 已完成并通过编译验证

---

## 一、修改概述

本次修改将 `CBoard` 类的通信方式从 **SocketCAN** 改为 **虚拟串口通信**，以实现更稳定、更易调试的数据传输。

### 核心变化
- **移除依赖**：`SocketCAN` → `serial::Serial`
- **数据传输**：CAN帧（8字节限制）→ 完整数据包（带帧头和CRC16）
- **通信方式**：基于CAN ID区分 → 基于帧头标识和结构化数据包

---

## 二、代码修改详情

### 2.1 修改的文件

| 文件 | 修改类型 | 说明 |
|------|---------|------|
| `io/cboard.hpp` | 重大修改 | 重新定义数据包结构，替换通信类 |
| `io/cboard.cpp` | 完全重写 | 实现串口通信逻辑 |
| `configs/example.yaml` | 配置更新 | 添加串口配置示例 |
| `configs/demo.yaml` | 配置更新 | 更新为串口配置 |
| `configs/standard3.yaml` | 配置更新 | 更新为串口配置 |
| `configs/standard4.yaml` | 配置更新 | 更新为串口配置 |

### 2.2 新增数据包结构

#### 下位机 → 视觉（CBoard_RX）
```cpp
struct __attribute__((packed)) CBoard_RX
{
  uint8_t head[2] = {'S', 'P'};  // 帧头标识
  float q[4];                     // 四元数 wxyz顺序
  float bullet_speed;             // 弹速 m/s
  uint8_t mode;                   // 模式: 0-idle, 1-auto_aim, 2-small_buff, 3-big_buff, 4-outpost
  uint8_t shoot_mode;             // 射击模式: 0-left, 1-right, 2-both
  float ft_angle;                 // FT角度（无人机专有）
  uint16_t crc16;                 // CRC16校验
};
```
**总大小**：32字节

#### 视觉 → 下位机（CBoard_TX）
```cpp
struct __attribute__((packed)) CBoard_TX
{
  uint8_t head[2] = {'S', 'P'};  // 帧头标识
  uint8_t control;                // 是否控制: 0-否, 1-是
  uint8_t shoot;                  // 是否射击: 0-否, 1-是
  float yaw;                      // yaw角度（弧度）
  float pitch;                    // pitch角度（弧度）
  float horizon_distance;         // 水平距离（无人机专有）
  uint16_t crc16;                 // CRC16校验
};
```
**总大小**：20字节

### 2.3 配置文件变更

**旧配置（CAN）**：
```yaml
quaternion_canid: 0x100
bullet_speed_canid: 0x101
send_canid: 0xff
can_interface: "can0"
```

**新配置（串口）**：
```yaml
com_port: "/dev/ttyACM0"  # 串口设备路径
baudrate: 115200          # 波特率
```

---

## 三、下位机需要的修改

### 3.1 通信协议实现

下位机需要实现对应的串口通信协议：

#### 发送数据（下位机 → 视觉）
```c
// 1. 填充数据
CBoard_RX tx_data;
tx_data.head[0] = 'S';
tx_data.head[1] = 'P';
tx_data.q[0] = q_w;  // 四元数 w
tx_data.q[1] = q_x;  // 四元数 x
tx_data.q[2] = q_y;  // 四元数 y
tx_data.q[3] = q_z;  // 四元数 z
tx_data.bullet_speed = 15.0f;  // 弹速（m/s）
tx_data.mode = 1;  // 模式（1=自瞄）
tx_data.shoot_mode = 0;  // 射击模式
tx_data.ft_angle = 0.0f;  // FT角度

// 2. 计算CRC16（不包括CRC字段）
tx_data.crc16 = crc16_calculate((uint8_t*)&tx_data, sizeof(tx_data) - 2);

// 3. 发送
uart_send((uint8_t*)&tx_data, sizeof(tx_data));
```

#### 接收数据（视觉 → 下位机）
```c
// 1. 查找帧头 'SP'
while (uart_read_byte() != 'S');
if (uart_read_byte() != 'P') continue;

// 2. 读取剩余数据
CBoard_TX rx_data;
rx_data.head[0] = 'S';
rx_data.head[1] = 'P';
uart_read((uint8_t*)&rx_data + 2, sizeof(rx_data) - 2);

// 3. CRC16校验
if (!crc16_check((uint8_t*)&rx_data, sizeof(rx_data))) {
    // 校验失败，丢弃
    continue;
}

// 4. 解析数据
bool control = (rx_data.control == 1);
bool shoot = (rx_data.shoot == 1);
float yaw = rx_data.yaw;      // 弧度
float pitch = rx_data.pitch;  // 弧度
```

### 3.2 CRC16算法参考

下位机需要实现与视觉端一致的CRC16算法。参考视觉端实现：
- 文件：`tools/crc.hpp` 和对应的实现文件
- 多项式：建议使用标准的CRC-16-CCITT（0x1021）或CRC-16-IBM（0x8005）

**提示**：可以查看项目中 `tools` 目录下的CRC实现，确保下位机使用相同算法。

### 3.3 串口配置

推荐配置：
- **波特率**：115200
- **数据位**：8
- **停止位**：1
- **校验位**：无
- **流控**：无

---

## 四、测试与验证

### 4.1 编译验证

所有程序已通过编译验证：
```bash
✅ io库编译成功
✅ cboard_test编译成功
✅ standard程序编译成功
```

### 4.2 测试流程

1. **下位机固件更新**
   - 实现新的串口通信协议
   - 烧录到C板

2. **连接测试**
   ```bash
   # 检查串口设备
   ls -l /dev/ttyACM*
   
   # 授予串口权限
   sudo usermod -a -G dialout $USER
   # 重新登录后生效
   ```

3. **运行测试程序**
   ```bash
   # 修改配置文件中的 com_port 为实际设备
   # 运行CBoard测试程序
   ./build/cboard_test configs/example.yaml
   ```

4. **预期输出**
   ```
   [CBoard] Serial port /dev/ttyACM0 opened at 115200 baud
   [CBoard] Waiting for first quaternion...
   [CBoard] CBoard opened successfully.
   [CBoard] Bullet speed: 15.00 m/s, Mode: auto_aim, ...
   ```

### 4.3 故障排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 无法打开串口 | 权限不足 | 执行 `sudo usermod -a -G dialout $USER` |
| CRC校验失败 | CRC算法不一致 | 确认下位机使用相同的CRC算法 |
| 接收不到数据 | 帧头识别失败 | 检查下位机是否正确发送'SP'帧头 |
| 四元数无效 | 数据格式错误 | 确认四元数归一化：w²+x²+y²+z²≈1 |

---

## 五、与旧CAN通信的对比

| 特性 | CAN通信 | 串口通信 |
|------|---------|----------|
| **数据包大小** | 8字节/帧 | 任意（本项目：20-32字节） |
| **数据完整性** | 需分多帧 | 一次传输完整数据 |
| **校验机制** | CAN自带CRC | 手动CRC16 |
| **帧同步** | 基于CAN ID | 基于帧头'SP' |
| **配置复杂度** | 需配置CAN网络 | 标准串口，简单 |
| **调试难度** | 需要专用工具 | 普通串口工具即可 |
| **稳定性** | 高 | 高（需正确实现） |

---

## 六、注意事项

### 6.1 重要提示

⚠️ **下位机协议必须同步更新**  
视觉端的修改依赖下位机实现相应的串口通信协议。下位机未更新前，视觉程序无法正常工作。

⚠️ **数据字节序**  
确保下位机与视觉端使用相同的字节序（通常为小端序 Little-Endian）。

⚠️ **四元数归一化**  
下位机发送的四元数必须归一化，否则视觉端会拒绝该数据。

### 6.2 udev规则固定串口（可选）

为避免设备路径变化，建议配置udev规则：
```bash
# 1. 获取设备信息
udevadm info -a -n /dev/ttyACM0 | grep -E '({serial}|{idVendor}|{idProduct})'

# 2. 创建udev规则
sudo nano /etc/udev/rules.d/99-usb-serial.rules

# 3. 添加规则（替换实际的ID）
SUBSYSTEM=="tty", ATTRS{idVendor}=="****", ATTRS{idProduct}=="****", SYMLINK+="cboard"

# 4. 重新加载
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后在配置文件中使用固定名称：
```yaml
com_port: "/dev/cboard"
```

---

## 七、后续工作

- [ ] 下位机固件更新和测试
- [ ] 实际硬件联调
- [ ] 性能测试（延迟、稳定性）
- [ ] 更新其他配置文件（如有需要）
- [ ] 完善文档和注释

---

## 八、技术支持

如遇问题，请检查：
1. 串口连接是否正常
2. 波特率配置是否一致
3. CRC算法是否匹配
4. 数据包结构是否对齐（`__attribute__((packed))`）
5. 四元数数据是否有效

参考代码：
- 视觉端：`io/cboard.cpp`
- 类似实现：`io/gimbal/gimbal.cpp`（可作为下位机参考）

---

**文档版本**：v1.0  
**最后更新**：2025年10月23日

