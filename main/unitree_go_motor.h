#ifndef UNITREE_GO_MOTOR_H
#define UNITREE_GO_MOTOR_H

#include <stdint.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// GO-M8010-6 协议常量
#define GO_M8010_6_CMD_LENGTH       17   // 命令帧长度
#define GO_M8010_6_DATA_LENGTH      16   // 反馈帧长度
#define GO_M8010_6_CMD_HEADER1      0xFE // 命令帧头1
#define GO_M8010_6_CMD_HEADER2      0xEE // 命令帧头2
#define GO_M8010_6_DATA_HEADER1     0xFD // 反馈帧头1
#define GO_M8010_6_DATA_HEADER2     0xEE // 反馈帧头2

// 电机配置结构体
typedef struct {
    uart_port_t uart_port;      // UART端口
    gpio_num_t tx_pin;          // TX引脚
    gpio_num_t rx_pin;          // RX引脚
    gpio_num_t de_re_pin;       // DE/RE控制引脚
    int baud_rate;              // 波特率
    uint8_t motor_id;           // 电机ID (0~14, 15为广播ID)
} unitree_motor_config_t;

// 电机命令结构体（使用官方命名）
typedef struct {
    float q;                // 期望角度(电机转子位置 单位: rad)
    float dq;               // 期望角速度(电机转子端转速 单位: rad/s)
    float tau;              // 前馈力矩 (N.m)
    float kp;               // 比例系数
    float kd;               // 阻尼系数
    uint8_t mode;           // 电机运行模式 (0-刹车, 1-FOC)
} unitree_motor_cmd_t;

// 电机反馈数据结构体（使用官方命名）
typedef struct {
    float q;                // 当前角度(电机转子位置 单位: rad)
    float dq;               // 当前角速度(电机转子端转速 单位: rad/s)
    float tau;              // 当前力矩 (N.m)
    int8_t temperature;     // 温度 (°C)
    uint8_t error;          // 错误状态
    uint8_t motor_id;       // 电机ID (0~14, 15为广播ID)
    uint8_t mode;           // 当前模式 (0-刹车, 1-FOC)
} unitree_motor_data_t;

// 电机通信统计
typedef struct {
    uint32_t cmd_count;      // 发送命令总数
    uint32_t success_count;  // 成功次数
    uint32_t error_count;    // 错误次数
} unitree_motor_stats_t;

/**
 * @brief 初始化宇树电机RS485通信
 * @param config 电机配置参数
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t unitree_motor_init(const unitree_motor_config_t *config);

/**
 * @brief 发送电机控制命令
 * @param config 电机配置参数
 * @param cmd 控制命令 (q-期望角度, dq-期望角速度, tau-前馈力矩, kp-比例系数, kd-阻尼系数, mode-运行模式)
 * @param data 反馈数据 (可为NULL，包含q-当前角度, dq-当前角速度, tau-当前力矩等)
 * @param stats 通信统计 (可为NULL)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t unitree_motor_send_recv(const unitree_motor_config_t *config, 
                                  const unitree_motor_cmd_t *cmd, 
                                  unitree_motor_data_t *data,
                                  unitree_motor_stats_t *stats);

/**
 * @brief 反初始化电机通信
 * @param config 电机配置参数
 */
void unitree_motor_deinit(const unitree_motor_config_t *config);

// ========== 便捷控制模式函数 ==========

/**
 * @brief 刹车模式 - 电机断电，自由转动
 * @param config 电机配置参数
 * @param data 反馈数据 (可为NULL)
 * @param stats 通信统计 (可为NULL)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t unitree_motor_brake_mode(const unitree_motor_config_t *config, 
                                   unitree_motor_data_t *data,
                                   unitree_motor_stats_t *stats);

/**
 * @brief 阻尼模式 - 提供阻尼力，无位置控制
 * @param config 电机配置参数
 * @param kd 阻尼系数 (0~25.599)
 * @param data 反馈数据 (可为NULL)
 * @param stats 通信统计 (可为NULL)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t unitree_motor_damping_mode(const unitree_motor_config_t *config,
                                     float kd,
                                     unitree_motor_data_t *data,
                                     unitree_motor_stats_t *stats);

/**
 * @brief 角度控制模式 - FOC位置控制
 * @param config 电机配置参数
 * @param target_angle 目标角度 (rad)
 * @param kp 比例系数 (推荐0.01)
 * @param kd 阻尼系数 (推荐0.01)
 * @param data 反馈数据 (可为NULL)
 * @param stats 通信统计 (可为NULL)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t unitree_motor_angle_mode(const unitree_motor_config_t *config,
                                   float target_angle,
                                   float kp,
                                   float kd,
                                   unitree_motor_data_t *data,
                                   unitree_motor_stats_t *stats);

/**
 * @brief 角速度控制模式 - FOC速度控制
 * @param config 电机配置参数
 * @param target_velocity 目标角速度 (rad/s)
 * @param kd 阻尼系数 (推荐0.01)
 * @param data 反馈数据 (可为NULL)
 * @param stats 通信统计 (可为NULL)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t unitree_motor_velocity_mode(const unitree_motor_config_t *config,
                                      float target_velocity,
                                      float kd,
                                      unitree_motor_data_t *data,
                                      unitree_motor_stats_t *stats);

/**
 * @brief 力矩控制模式 - FOC直接力矩输出
 * @param config 电机配置参数
 * @param target_torque 目标力矩 (N.m, -127.99~127.99)
 * @param data 反馈数据 (可为NULL)
 * @param stats 通信统计 (可为NULL)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t unitree_motor_torque_mode(const unitree_motor_config_t *config,
                                    float target_torque,
                                    unitree_motor_data_t *data,
                                    unitree_motor_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // UNITREE_GO_MOTOR_H