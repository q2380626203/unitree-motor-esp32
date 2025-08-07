/*
 * 宇树GO-M8010-6电机控制简化示例
 * 
 * 使用便捷的模式函数，让电机控制更简单！
 */

#include "unitree_go_motor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "SIMPLE_MOTOR";

void simple_motor_demo(void *arg)
{
    // 1. 电机配置
    unitree_motor_config_t motor_config = {
        .uart_port = UART_NUM_2,        // 使用UART2
        .tx_pin = GPIO_NUM_17,          // TX引脚
        .rx_pin = GPIO_NUM_16,          // RX引脚  
        .de_re_pin = GPIO_NUM_4,        // DE/RE控制引脚
        .baud_rate = 4000000,           // 4M波特率
        .motor_id = 0                   // 电机ID
    };
    
    // 2. 初始化电机通信
    esp_err_t ret = unitree_motor_init(&motor_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "电机初始化失败: %d", ret);
        vTaskDelete(NULL);
        return;
    }
    
    unitree_motor_data_t motor_data = {0};
    unitree_motor_stats_t motor_stats = {0};
    
    ESP_LOGI(TAG, "=== 宇树电机简化控制演示 ===");
    
    // 演示1: 刹车模式 (1秒)
    ESP_LOGI(TAG, "1. 刹车模式 - 电机断电自由转动");
    for (int i = 0; i < 100; i++) { 
        unitree_motor_brake_mode(&motor_config, &motor_data, &motor_stats);
        if (i % 25 == 0) {
            ESP_LOGI(TAG, "刹车模式 - 角度: %.2f rad, 角速度: %.2f rad/s", 
                     motor_data.q, motor_data.dq);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
    
    // 演示2: 阻尼模式 (2秒)
    ESP_LOGI(TAG, "2. 阻尼模式 - 提供阻尼力，手动转动有阻力");
    for (int i = 0; i < 200; i++) { 
        unitree_motor_damping_mode(&motor_config, 1.0f, &motor_data, &motor_stats); // kd=1.0
        if (i % 50 == 0) {
            ESP_LOGI(TAG, "阻尼模式 - 角度: %.2f rad, 角速度: %.2f rad/s", 
                     motor_data.q, motor_data.dq);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // 演示3: 角度控制 - 移动到不同位置
    ESP_LOGI(TAG, "3. 角度控制 - 精确位置控制");
    float target_angles[] = {0.0f, 1.57f, -1.57f, 0.0f}; // 0, π/2, -π/2, 0
    
    for (int angle_idx = 0; angle_idx < 4; angle_idx++) {
        ESP_LOGI(TAG, "移动到角度: %.2f rad", target_angles[angle_idx]);
        
        for (int i = 0; i < 150; i++) { // 1.5秒到达每个位置
            unitree_motor_angle_mode(&motor_config, target_angles[angle_idx], 
                                    0.01f, 0.01f, &motor_data, &motor_stats);
            if (i % 50 == 0) {
                ESP_LOGI(TAG, "角度控制 - 目标: %.2f, 当前: %.2f rad", 
                         target_angles[angle_idx], motor_data.q);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    // 演示4: 角速度控制
    ESP_LOGI(TAG, "4. 角速度控制 - 恒定转速");
    float target_velocities[] = {2.0f, -2.0f, 0.0f}; // 2, -2, 0 rad/s
    
    for (int vel_idx = 0; vel_idx < 3; vel_idx++) {
        ESP_LOGI(TAG, "设置角速度: %.2f rad/s", target_velocities[vel_idx]);
        
        for (int i = 0; i < 100; i++) { // 1秒
            unitree_motor_velocity_mode(&motor_config, target_velocities[vel_idx], 
                                       0.01f, &motor_data, &motor_stats);
            if (i % 25 == 0) {
                ESP_LOGI(TAG, "速度控制 - 目标: %.2f, 当前: %.2f rad/s", 
                         target_velocities[vel_idx], motor_data.dq);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    // 演示5: 力矩控制
    ESP_LOGI(TAG, "5. 力矩控制 - 直接力矩输出");
    float target_torques[] = {0.3f, -0.3f, 0.0f}; // 0.3, -0.3, 0 N.m
    
    for (int torque_idx = 0; torque_idx < 3; torque_idx++) {
        ESP_LOGI(TAG, "设置力矩: %.2f N.m", target_torques[torque_idx]);
        
        for (int i = 0; i < 50; i++) { // 0.5秒
            unitree_motor_torque_mode(&motor_config, target_torques[torque_idx], 
                                     &motor_data, &motor_stats);
            if (i % 10 == 0) {
                ESP_LOGI(TAG, "力矩控制 - 目标: %.2f, 当前: %.2f N.m", 
                         target_torques[torque_idx], motor_data.tau);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    // 最终回到刹车模式
    ESP_LOGI(TAG, "演示完成，回到刹车模式");
    for (int i = 0; i < 50; i++) { 
        unitree_motor_brake_mode(&motor_config, &motor_data, &motor_stats);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // 统计信息
    ESP_LOGI(TAG, "=== 演示完成 ===");
    ESP_LOGI(TAG, "总命令数: %lu", motor_stats.cmd_count);
    ESP_LOGI(TAG, "成功次数: %lu", motor_stats.success_count);
    ESP_LOGI(TAG, "成功率: %.1f%%", 100.0f * motor_stats.success_count / motor_stats.cmd_count);
    
    // 清理资源
    unitree_motor_deinit(&motor_config);
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "宇树电机简化控制演示启动");
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 创建演示任务
    xTaskCreate(simple_motor_demo, "simple_motor_demo", 4096, NULL, 5, NULL);
}