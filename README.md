# 宇树GO-M8010-6电机ESP32控制库

基于ESP32通过RS485协议控制宇树GO-M8010-6电机的C/C++库。

## 特性

- 🎯 **简单易用**: 5个便捷函数涵盖所有控制模式
- 🔧 **模块化设计**: 独立封装，易于集成到现有项目
- ⚡ **高性能**: 支持最高1kHz控制频率，实时反馈
- 🛡️ **稳定可靠**: 完善的CRC校验和错误处理

## 硬件连接

```
ESP32      MAX13487     宇树电机
GPIO17  -> DI           
GPIO16  <- RO           
GPIO4   -> DE/RE        
          A <-------> A
          B <-------> B
```

## 快速开始

### 1. 初始化
```c
#include "unitree_go_motor.h"

unitree_motor_config_t config = {
    .uart_port = UART_NUM_2,
    .tx_pin = GPIO_NUM_17,
    .rx_pin = GPIO_NUM_16,
    .de_re_pin = GPIO_NUM_4,
    .baud_rate = 4000000,
    .motor_id = 0
};

unitree_motor_init(&config);
```

### 2. 控制电机
```c
unitree_motor_data_t data;
unitree_motor_stats_t stats;

// 角度控制 - 转到90度
unitree_motor_angle_mode(&config, 1.57f, 0.01f, 0.01f, &data, &stats);

// 速度控制 - 2rad/s转速
unitree_motor_velocity_mode(&config, 2.0f, 0.01f, &data, &stats);

// 力矩控制 - 输出0.5N.m力矩
unitree_motor_torque_mode(&config, 0.5f, &data, &stats);

// 阻尼模式 - 提供阻尼
unitree_motor_damping_mode(&config, 1.0f, &data, &stats);

// 刹车模式 - 电机断电
unitree_motor_brake_mode(&config, &data, &stats);
```

## 编译使用

```bash
# ESP-IDF项目
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

# 集成到项目 - 复制这些文件:
# unitree_go_motor.h/.c, crc_utils.h/.cpp
```

## 项目结构

```
├── main/
│   ├── rs485_example.cpp          # 主程序示例
│   ├── unitree_go_motor.h/.c      # 核心库文件  
│   └── crc_utils.h/.cpp           # CRC校验
└── simple_example.cpp             # 使用示例
```

## API参考

### 便捷控制函数
```c
esp_err_t unitree_motor_angle_mode(config, angle, kp, kd, data, stats);
esp_err_t unitree_motor_velocity_mode(config, velocity, kd, data, stats);  
esp_err_t unitree_motor_torque_mode(config, torque, data, stats);
esp_err_t unitree_motor_damping_mode(config, kd, data, stats);
esp_err_t unitree_motor_brake_mode(config, data, stats);
```

### 底层控制函数  
```c
esp_err_t unitree_motor_init(const unitree_motor_config_t *config);
esp_err_t unitree_motor_send_recv(config, cmd, data, stats);
void unitree_motor_deinit(const unitree_motor_config_t *config);
```

## 推荐参数

- **控制频率**: 100Hz (10ms周期)
- **kp**: 0.01 (角度控制)  
- **kd**: 0.01 (速度/阻尼控制)
- **波特率**: 4000000

## 许可协议

基于ESP-IDF示例代码修改，保持原有开源协议。