/*
 * Unitree Motor RS485 Communication Example
 * 
 * This example demonstrates RS485 communication with Unitree GO-M8010-6 motors
 * Features:
 * - Motor data parsing and command sending
 * - High-speed 4M baud rate communication
 * - RS485 half-duplex mode with MAX13487 transceiver
 * - Real-time motor status monitoring and control
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Unitree GO motor protocol
#include "unitree_go_motor.h"

#define TAG "UNITREE_MOTOR_RS485"

// Pin configuration from Kconfig
#define MOTOR_UART_TXD          (CONFIG_ECHO_UART_TXD)
#define MOTOR_UART_RXD          (CONFIG_ECHO_UART_RXD)
#define MOTOR_UART_RTS          (CONFIG_ECHO_UART_RTS)  // DE/RE pin for MAX13487
#define MOTOR_BAUD_RATE         (CONFIG_ECHO_UART_BAUD_RATE)
#define MOTOR_UART_PORT         (CONFIG_ECHO_UART_PORT_NUM)

// Task parameters
#define MOTOR_TASK_STACK_SIZE   (CONFIG_ECHO_TASK_STACK_SIZE)
#define MOTOR_TASK_PRIO         (10)

// Motor control parameters
#define MOTOR_ID                0    // 电机ID (与原工作代码一致)
#define CONTROL_FREQ_HZ         100  // 控制频率100Hz
#define CONTROL_PERIOD_MS       (1000 / CONTROL_FREQ_HZ)


// 电机主动控制任务 - 使用模块化的协议实现
static void motor_control_task(void *arg)
{
    ESP_LOGI(TAG, "启动宇树电机RS485主动控制系统");
    
    // 电机配置
    unitree_motor_config_t motor_config = {
        .uart_port = (uart_port_t)MOTOR_UART_PORT,
        .tx_pin = (gpio_num_t)MOTOR_UART_TXD,
        .rx_pin = (gpio_num_t)MOTOR_UART_RXD,
        .de_re_pin = (gpio_num_t)MOTOR_UART_RTS,
        .baud_rate = MOTOR_BAUD_RATE,
        .motor_id = MOTOR_ID
    };
    
    ESP_LOGI(TAG, "引脚配置 - TX: GPIO%d, RX: GPIO%d, DE/RE: GPIO%d", 
             motor_config.tx_pin, motor_config.rx_pin, motor_config.de_re_pin);
    ESP_LOGI(TAG, "波特率: %d, 控制频率: 100Hz", motor_config.baud_rate);

    // 初始化电机通信
    esp_err_t ret = unitree_motor_init(&motor_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机通信初始化失败: %d", ret);
        vTaskDelete(NULL);
        return;
    }

    // 通信统计
    unitree_motor_stats_t motor_stats = {0};
    
    // 电机控制参数（使用官方命名）
    unitree_motor_cmd_t motor_cmd = {
        .q = 0.0f,            // 期望角度 rad
        .dq = 0.0f,           // 期望角速度 rad/s  
        .tau = 0.0f,          // 前馈力矩 N.m
        .kp = 0.0f,           // 比例系数
        .kd = 0.0f,           // 阻尼系数
        .mode = 0             // FOC模式
    };
    
    unitree_motor_data_t motor_data = {0};
    
    ESP_LOGI(TAG, "开始发送电机空闲指令 - 让电机进入安全状态");
    
    while (1) {
        // 发送控制指令并接收反馈
        esp_err_t result = unitree_motor_send_recv(&motor_config, &motor_cmd, &motor_data, &motor_stats);
        
        // 状态显示
        if (motor_stats.cmd_count % 50 == 0) { // 每500ms显示一次
            ESP_LOGI(TAG, "========== 电机状态 #%lu ==========", motor_stats.cmd_count);
            ESP_LOGI(TAG, "控制指令: Kp=%.1f, Kd=%.1f, tau=%.3f, q=%.3f, dq=%.2f", 
                     motor_cmd.kp, motor_cmd.kd, motor_cmd.tau, motor_cmd.q, motor_cmd.dq);
            
            if (result == ESP_OK) {
                ESP_LOGI(TAG, "当前角度: %.3f rad (%.1f°)", motor_data.q, motor_data.q * 57.2958f);
                ESP_LOGI(TAG, "当前角速度: %.2f rad/s", motor_data.dq);
                ESP_LOGI(TAG, "当前力矩: %.3f N.m", motor_data.tau);
                ESP_LOGI(TAG, "温度: %d°C, 错误: 0x%02X", motor_data.temperature, motor_data.error);
            } else {
                ESP_LOGW(TAG, "通信失败");
            }
            
            ESP_LOGI(TAG, "成功率: %lu/%lu (%.1f%%)", motor_stats.success_count, 
                     motor_stats.cmd_count, 100.0f * motor_stats.success_count / motor_stats.cmd_count);
            ESP_LOGI(TAG, "=====================================");
        }
        
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS)); // 100Hz控制频率
    }
    
    // 清理资源
    unitree_motor_deinit(&motor_config);
    
    vTaskDelete(NULL);
}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "宇树电机RS485通信系统启动 (主动控制模式)");
    
    // 初始化NVS (如果需要保存配置)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 创建电机主动控制任务
    xTaskCreate(motor_control_task, "motor_control", MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIO, NULL);
    
    ESP_LOGI(TAG, "电机控制任务已创建，开始主动控制电机");
}