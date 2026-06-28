#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO     14    // Your SDA pin
#define I2C_MASTER_SCL_IO     13    // Your SCL pin
#define M5_RELAY_ADDR         0x26  // Relay I2C address
#define RELAY_REG             0x10  // Target control register

static const char *TAG = "M5_RELAY";

void app_main(void)
{
    // 1. Configure the I2C Master Bus
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    // 2. Add the Relay Device to the Bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = M5_RELAY_ADDR,
        .scl_speed_hz = 100000, // 100 kHz standard mode
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    ESP_LOGI(TAG, "I2C Initialized on SDA: %d, SCL: %d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    while (1) {
        // Buffer payload: [Register Address, Relay Mask Value]
        uint8_t turn_on_data[2] = {RELAY_REG, 0x0F};  // 0x0F (0b1111) -> All 4 relays ON
        uint8_t turn_off_data[2] = {RELAY_REG, 0x00}; // 0x00 (0b0000) -> All 4 relays OFF

        ESP_LOGI(TAG, "Turning ALL relays ON");
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, turn_on_data, sizeof(turn_on_data), -1));
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2 seconds

        ESP_LOGI(TAG, "Turning ALL relays OFF");
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, turn_off_data, sizeof(turn_off_data), -1));
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2 seconds
    }
}