#include "unitree_go_motor.h"
#include "crc_utils.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "UNITREE_GO_MOTOR";

// 打印原始数据包的辅助函数
static void print_raw_packet(const char* label, const uint8_t* data, int length) {
    printf("[%s] 原始数据包(%d字节): ", label, length);
    for (int i = 0; i < length; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 8 == 0 && i < length - 1) {
            printf("\n[%s]                      ", label);
        }
    }
    printf("\n");
}

// 搜索帧头的辅助函数
static int find_frame_header(const uint8_t* buffer, int buffer_len, uint8_t header1, uint8_t header2) {
    for (int i = 0; i < buffer_len - 1; i++) {
        if (buffer[i] == header1 && buffer[i + 1] == header2) {
            return i;  // 返回帧头位置
        }
    }
    return -1;  // 未找到帧头
}

esp_err_t unitree_motor_init(const unitree_motor_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "配置参数为空");
        return ESP_ERR_INVALID_ARG;
    }

    // 配置DE/RE引脚为输出
    gpio_config_t de_re_config = {
        .pin_bit_mask = (1ULL << config->de_re_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&de_re_config));
    gpio_set_level(config->de_re_pin, 0); // 默认接收模式
    
    // UART配置
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };

    // 安装UART驱动
    esp_err_t ret = uart_driver_install(config->uart_port, 256, 256, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART驱动安装失败: %d", ret);
        return ret;
    }

    ret = uart_param_config(config->uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART参数配置失败: %d", ret);
        uart_driver_delete(config->uart_port);
        return ret;
    }

    ret = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART引脚配置失败: %d", ret);
        uart_driver_delete(config->uart_port);
        return ret;
    }

    ret = uart_set_rx_timeout(config->uart_port, 10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART接收超时设置失败: %d", ret);
        uart_driver_delete(config->uart_port);
        return ret;
    }

    ESP_LOGI(TAG, "电机通信初始化完成 - UART%d, TX:%d, RX:%d, DE/RE:%d, 波特率:%d", 
             config->uart_port, config->tx_pin, config->rx_pin, 
             config->de_re_pin, config->baud_rate);

    return ESP_OK;
}

esp_err_t unitree_motor_send_recv(const unitree_motor_config_t *config, 
                                  const unitree_motor_cmd_t *cmd, 
                                  unitree_motor_data_t *data,
                                  unitree_motor_stats_t *stats)
{
    if (!config || !cmd) {
        ESP_LOGE(TAG, "参数为空");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd_packet[GO_M8010_6_CMD_LENGTH];
    uint8_t rx_buffer[32];

    // ========== 构建宇树标准命令数据包 ==========
    
    cmd_packet[0] = GO_M8010_6_CMD_HEADER1; // 帧头
    cmd_packet[1] = GO_M8010_6_CMD_HEADER2; 
    cmd_packet[2] = (config->motor_id & 0x0F) | ((cmd->mode & 0x07) << 4); // ID + MODE
    
    // tau: 前馈力矩 (N.m * 256)
    int16_t tau_cmd = (int16_t)(cmd->tau * 256.0f);
    cmd_packet[3] = (uint8_t)(tau_cmd & 0xFF);        // 低位
    cmd_packet[4] = (uint8_t)((tau_cmd >> 8) & 0xFF); // 高位
    
    // dq: 期望角速度 (rad/s * 256/2π)  
    int16_t dq_cmd = (int16_t)(cmd->dq * 256.0f / 6.28318f);
    cmd_packet[5] = (uint8_t)(dq_cmd & 0xFF);        // 低位
    cmd_packet[6] = (uint8_t)((dq_cmd >> 8) & 0xFF); // 高位
    
    // q: 期望角度 (rad * 32768/2π)
    int32_t q_cmd = (int32_t)(cmd->q * 32768.0f / 6.28318f);
    cmd_packet[7] = (uint8_t)(q_cmd & 0xFF);         // 最低位
    cmd_packet[8] = (uint8_t)((q_cmd >> 8) & 0xFF);  // 次低位
    cmd_packet[9] = (uint8_t)((q_cmd >> 16) & 0xFF); // 次高位
    cmd_packet[10] = (uint8_t)((q_cmd >> 24) & 0xFF);// 最高位
    
    // Kp: 位置刚度 (kp * 32768/25.6)
    uint16_t kp_cmd = (uint16_t)(cmd->kp * 32768.0f / 25.6f);
    cmd_packet[11] = (uint8_t)(kp_cmd & 0xFF);        // 低位
    cmd_packet[12] = (uint8_t)((kp_cmd >> 8) & 0xFF); // 高位
    
    // Kd: 速度阻尼 (kd * 32768/25.6) 
    uint16_t kd_cmd = (uint16_t)(cmd->kd * 32768.0f / 25.6f);
    cmd_packet[13] = (uint8_t)(kd_cmd & 0xFF);        // 低位
    cmd_packet[14] = (uint8_t)((kd_cmd >> 8) & 0xFF); // 高位
    
    // CRC计算 (前15字节)
    uint16_t crc = crc_ccitt(0x0000, cmd_packet, 15);
    cmd_packet[15] = (uint8_t)(crc & 0xFF);        // CRC低位
    cmd_packet[16] = (uint8_t)((crc >> 8) & 0xFF); // CRC高位
    
    // 打印发送的命令数据包
    print_raw_packet("发送CMD", cmd_packet, GO_M8010_6_CMD_LENGTH);
    
    // ========== RS485半双工发送 ==========
    
    gpio_set_level(config->de_re_pin, 1); // 切换到发送模式
    vTaskDelay(pdMS_TO_TICKS(1)); // 短暂延时确保切换完成
    
    int tx_len = uart_write_bytes(config->uart_port, cmd_packet, GO_M8010_6_CMD_LENGTH);
    uart_wait_tx_done(config->uart_port, pdMS_TO_TICKS(20)); // 增加等待发送完成时间
    
    vTaskDelay(pdMS_TO_TICKS(1)); // 额外延时确保数据完全发送
    gpio_set_level(config->de_re_pin, 0); // 切换到接收模式
    
    // 更新统计信息
    if (stats) {
        stats->cmd_count++;
    }
    
    if (tx_len != GO_M8010_6_CMD_LENGTH) {
        ESP_LOGW(TAG, "发送失败: %d/%d 字节", tx_len, GO_M8010_6_CMD_LENGTH);
        if (stats) stats->error_count++;
        return ESP_FAIL;
    }
    
    // ========== 接收反馈 ==========
    
    // 清空UART接收缓冲区，避免残留数据干扰
    uart_flush_input(config->uart_port);
    
    vTaskDelay(pdMS_TO_TICKS(2)); // 增加等待时间，确保电机响应完成
    
    // 读取更多数据用于帧头搜索
    uint8_t extended_buffer[32];
    int rx_len = uart_read_bytes(config->uart_port, extended_buffer, sizeof(extended_buffer), pdMS_TO_TICKS(10));
    
    // 打印接收到的原始数据包
    if (rx_len > 0) {
        print_raw_packet("接收DATA", extended_buffer, rx_len);
    } else {
        printf("[接收DATA] 未接收到数据 (长度: %d)\n", rx_len);
    }
    
    // 搜索正确的帧头
    int header_pos = find_frame_header(extended_buffer, rx_len, GO_M8010_6_DATA_HEADER1, GO_M8010_6_DATA_HEADER2);
    
    if (header_pos >= 0 && (header_pos + GO_M8010_6_DATA_LENGTH <= rx_len)) {
        // 找到帧头，复制正确的数据包
        memcpy(rx_buffer, extended_buffer + header_pos, GO_M8010_6_DATA_LENGTH);
        
        if (header_pos > 0) {
            printf("[帧同步] 在位置%d找到帧头，跳过%d字节垃圾数据\n", header_pos, header_pos);
        }
        
        // ========== 解析反馈数据 ==========
        
        // 打印详细解析过程
        uint8_t motor_id = rx_buffer[2] & 0x0F;
        uint8_t mode = (rx_buffer[2] >> 4) & 0x07;
        
        int16_t tau_raw = (int16_t)(rx_buffer[4] << 8) | rx_buffer[3];
        float tau = (float)tau_raw / 256.0f;
        
        int16_t dq_raw = (int16_t)(rx_buffer[6] << 8) | rx_buffer[5];
        float dq = (float)dq_raw * (6.28318f / 256.0f);
        
        int32_t q_raw = (int32_t)(rx_buffer[10] << 24) | (rx_buffer[9] << 16) | 
                       (rx_buffer[8] << 8) | rx_buffer[7];
        float q = (float)q_raw * (6.28318f / 32768.0f);
        
        int8_t temperature = (int8_t)rx_buffer[11];
        uint8_t error = rx_buffer[12] & 0x07;
        
        printf("[解析结果] ID:%d, 模式:%d, 角度:%.3f rad, 角速度:%.2f rad/s, 力矩:%.3f N.m, 温度:%d°C, 错误:0x%02X\n",
               motor_id, mode, q, dq, tau, temperature, error);
        printf("[原始数值] tau_raw:%d, dq_raw:%d, q_raw:%" PRId32 "\n", tau_raw, dq_raw, q_raw);
        
        if (data) {
            data->motor_id = motor_id;
            data->mode = mode;
            data->tau = tau;
            data->dq = dq;
            data->q = q;
            data->temperature = temperature;
            data->error = error;
        }
        
        if (stats) stats->success_count++;
        return ESP_OK;
        
    } else {
        printf("[接收失败] 长度:%d (期望:%d), 帧头: %02X %02X (期望: %02X %02X)\n", 
               rx_len, GO_M8010_6_DATA_LENGTH,
               rx_len > 0 ? rx_buffer[0] : 0, rx_len > 1 ? rx_buffer[1] : 0,
               GO_M8010_6_DATA_HEADER1, GO_M8010_6_DATA_HEADER2);
        
        if (rx_len > 0) {
            printf("[错误数据] ");
            for (int i = 0; i < rx_len && i < 16; i++) {
                printf("%02X ", rx_buffer[i]);
            }
            printf("\n");
        }
        
        if (stats) stats->error_count++;
        return ESP_FAIL;
    }
}

void unitree_motor_deinit(const unitree_motor_config_t *config)
{
    if (config) {
        uart_driver_delete(config->uart_port);
        gpio_reset_pin(config->de_re_pin);
        ESP_LOGI(TAG, "电机通信反初始化完成");
    }
}

// ========== 便捷控制模式函数实现 ==========

esp_err_t unitree_motor_brake_mode(const unitree_motor_config_t *config, 
                                   unitree_motor_data_t *data,
                                   unitree_motor_stats_t *stats)
{
    unitree_motor_cmd_t cmd = {
        .q = 0.0f,            // 不控制角度     
        .dq = 0.0f,           // 不控制角速度     
        .tau = 0.0f,          // 无前馈力矩       
        .kp = 0.0f,           // 不使用角度控制
        .kd = 0.0f,           // 不使用阻尼
        .mode = 0             // 刹车模式
    };
    
    return unitree_motor_send_recv(config, &cmd, data, stats);
}

esp_err_t unitree_motor_damping_mode(const unitree_motor_config_t *config,
                                     float kd,
                                     unitree_motor_data_t *data,
                                     unitree_motor_stats_t *stats)
{
    unitree_motor_cmd_t cmd = {
        .q = 0.0f,            // 不控制角度     
        .dq = 0.0f,           // 不控制角速度     
        .tau = 0.0f,          // 无前馈力矩       
        .kp = 0.0f,           // 不使用角度控制
        .kd = kd,             // 设置阻尼系数
        .mode = 1             // FOC模式
    };
    
    return unitree_motor_send_recv(config, &cmd, data, stats);
}

esp_err_t unitree_motor_angle_mode(const unitree_motor_config_t *config,
                                   float target_angle,
                                   float kp,
                                   float kd,
                                   unitree_motor_data_t *data,
                                   unitree_motor_stats_t *stats)
{
    unitree_motor_cmd_t cmd = {
        .q = target_angle,    // 目标角度
        .dq = 0.0f,           // 不控制角速度     
        .tau = 0.0f,          // 无前馈力矩       
        .kp = kp,             // 比例系数
        .kd = kd,             // 阻尼系数
        .mode = 1             // FOC模式
    };
    
    return unitree_motor_send_recv(config, &cmd, data, stats);
}

esp_err_t unitree_motor_velocity_mode(const unitree_motor_config_t *config,
                                      float target_velocity,
                                      float kd,
                                      unitree_motor_data_t *data,
                                      unitree_motor_stats_t *stats)
{
    unitree_motor_cmd_t cmd = {
        .q = 0.0f,            // 目标角度填0     
        .dq = target_velocity,// 目标角速度     
        .tau = 0.0f,          // 无前馈力矩      
        .kp = 0.0f,           // 不使用角度控制
        .kd = kd,             // 阻尼系数
        .mode = 1             // FOC模式
    };
    
    return unitree_motor_send_recv(config, &cmd, data, stats);
}

esp_err_t unitree_motor_torque_mode(const unitree_motor_config_t *config,
                                    float target_torque,
                                    unitree_motor_data_t *data,
                                    unitree_motor_stats_t *stats)
{
    unitree_motor_cmd_t cmd = {
        .q = 0.0f,            // 不控制角度     
        .dq = 0.0f,           // 不控制角速度     
        .tau = target_torque, // 目标力矩       
        .kp = 0.0f,           // 不使用角度控制
        .kd = 0.0f,           // 不使用阻尼
        .mode = 1             // FOC模式
    };
    
    return unitree_motor_send_recv(config, &cmd, data, stats);
}