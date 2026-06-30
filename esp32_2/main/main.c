#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_SDA_IO     21    // ESP32 default I2C SDA pin
#define I2C_MASTER_SCL_IO     22    // ESP32 default I2C SCL pin
#define M5_RELAY_ADDR         0x26  // Official M5Stack M121 relay module address
#define RELAY_REG             0x10  // Official relay control register
#define I2C_SCL_SPEED_HZ      100000 // Official examples use 200 kHz

static const char *TAG = "M5_RELAY";

static void scan_i2c_bus(i2c_master_bus_handle_t bus_handle)
{
    int found = 0;
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        esp_err_t err = i2c_master_probe(bus_handle, addr, 100);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  I2C device found at 0x%02x", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found on bus");
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s) on bus", found);
    }
}

void app_main(void)
{
    esp_err_t err;

    // 1. Configure the I2C Master Bus
    i2c_master_bus_config_t scan_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle = NULL;

    err = i2c_new_master_bus(&scan_bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "I2C bus configured on SDA:%d SCL:%d", scan_bus_config.sda_io_num, scan_bus_config.scl_io_num);
    ESP_LOGI(TAG, "Use VCC=5V, GND=GND, common ground with ESP32, and check SDA/SCL pull-ups if needed");

    while (1) {
        scan_i2c_bus(bus_handle);

        err = i2c_master_probe(bus_handle, M5_RELAY_ADDR, 1000);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Assumed relay address 0x%02x responded", M5_RELAY_ADDR);
        } else {
            ESP_LOGW(TAG, "Assumed relay address 0x%02x did not respond: %s", M5_RELAY_ADDR, esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}