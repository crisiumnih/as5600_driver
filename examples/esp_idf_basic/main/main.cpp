#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "as5600.hpp"

static const char *TAG = "AS5600_Example";

// I2C Configuration
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// Platform-specific I2C functions for ESP-IDF
AS5600_RET_TYPE esp_i2c_read(void* handle, uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK) ? AS5600_RET_OK : AS5600_RET_I2C_FAIL;
}

AS5600_RET_TYPE esp_i2c_write(void* handle, uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, buf, len, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK) ? AS5600_RET_OK : AS5600_RET_I2C_FAIL;
}

AS5600_RET_TYPE esp_delay_ms(void* handle, uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
    return AS5600_RET_OK;
}

// Initialize I2C master
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                             I2C_MASTER_RX_BUF_DISABLE, 
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== AS5600 ESP-IDF Example ===");
    
    // Initialize I2C
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");
    
    // Create AS5600 instance
    AS5600 encoder(nullptr, esp_i2c_read, esp_i2c_write, esp_delay_ms);
    
    // Initialize encoder
    ESP_LOGI(TAG, "Initializing AS5600...");
    AS5600_RET_TYPE ret = encoder.Init();
    if (ret != AS5600_RET_OK) {
        ESP_LOGE(TAG, "Failed to initialize AS5600 (error: %d)", ret);
        return;
    }
    ESP_LOGI(TAG, "AS5600 initialized successfully");
    
    // Check magnet status
    if (!encoder.MagnetDetected()) {
        ESP_LOGW(TAG, "No magnet detected!");
    } else if (encoder.MagnetTooWeak()) {
        ESP_LOGW(TAG, "Magnet too weak, move closer");
    } else if (encoder.MagnetTooStrong()) {
        ESP_LOGW(TAG, "Magnet too strong, move away");
    } else {
        ESP_LOGI(TAG, "Magnet position: OK");
    }
    
    ESP_LOGI(TAG, "Starting angle reading...\n");
    
    // Main loop
    while (1) {
        uint16_t raw;
        float degrees, normalized;
        int16_t delta;
        
        if (encoder.ReadRawAngle(&raw) == AS5600_RET_OK) {
            encoder.ReadAngleDeg(&degrees);
            encoder.ReadAngle01(&normalized);
            delta = encoder.GetDelta();
            
            ESP_LOGI(TAG, "Raw: %4d | Deg: %6.2f° | Norm: %.4f | Delta: %+5d", 
                     raw, degrees, normalized, delta);
        } else {
            ESP_LOGE(TAG, "Failed to read angle");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
