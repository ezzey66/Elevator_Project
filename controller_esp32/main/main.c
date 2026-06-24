#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"

// Receiver #2's REAL MAC address (The one with the sensor)
uint8_t receiver_mac[] = {0x30, 0x76, 0xF5, 0xF7, 0x57, 0x48};

// 1. New Callback: This fires automatically whenever Board #2 sends a message
void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // Safely format the incoming data as a string
    char incoming_msg[32];
    snprintf(incoming_msg, sizeof(incoming_msg), "%.*s", len, data);

    printf("\n=========================================\n");
    printf("Incoming Wireless Signal: %s\n", incoming_msg);

    // Act based on what the sensor detected
    if (strcmp(incoming_msg, "CALL_ELEVATOR_FLOOR_2") == 0) {
        printf("ACTION: Robot detected at Floor 2! Dispatching elevator...\n");
        // Future code to move the elevator goes here!
    } 
    else if (strcmp(incoming_msg, "FLOOR_2_CLEAR") == 0) {
        printf("STATUS: Floor 2 area is now clear.\n");
    }
    printf("=========================================\n");
}

// Optional callback left in case the controller needs to talk back later
void on_data_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        printf("Delivery Status: Success\n");
    } else {
        printf("Delivery Status: FAIL\n");
    }
}

void app_main(void) {
    // 2. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 3. Initialize Wi-Fi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 4. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // 5. Register BOTH callbacks (Sending and Receiving)
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv)); // <-- Critical line!

    // 6. Register the Peer (Make sure it uses Channel 1 to match Board 2)
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, receiver_mac, 6);
    peer_info.channel = 1; // Match Board 2's channel
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        printf("Failed to add peer!\n");
        return;
    }

    printf("Elevator Control Unit Online. Waiting for wireless requests...\n");
    
    // 7. Silent waiting loop
    while (1) {
        // The controller just waits here. The on_data_recv function 
        // does all the heavy lifting when a packet arrives.
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}