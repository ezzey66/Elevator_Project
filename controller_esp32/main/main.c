#include <stdio.h>
#include <string.h>
#include <stdbool.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"

// Receiver's MAC address (Board #2)
uint8_t receiver_mac[] = {0x30, 0x76, 0xF5, 0xF7, 0x57, 0x48};

// State Variables to prevent print-spamming
volatile bool elevator_called = false;
int state_timer = 0;

// 1. Updated Callback: Tracks connections and switch events
void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    char incoming_msg[32];
    snprintf(incoming_msg, sizeof(incoming_msg), "%.*s", len, data);

    // --- NEW: Handle Bootup Handshake Connection Message ---
    if (strcmp(incoming_msg, "BOARD_2_CONNECTED") == 0) {
        printf("\n=========================================\n");
        printf("Packet Received From: %02X:%02X:%02X:%02X:%02X:%02X\n",
               recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
               recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
        printf("STATUS: Board #2 (Floor 2 Station) has successfully CONNECTED!\n");
        printf("Wireless Link established on Channel 1.\n");
        printf("=========================================\n");
    }
    // --- Handle Magnet Switch Triggers ---
    else if (strcmp(incoming_msg, "CALL_ELEVATOR_FLOOR_2") == 0) {
        if (!elevator_called) {
            printf("\n=========================================\n");
            printf("Packet Received From: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                   recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
            printf("Incoming Wireless Signal: %s\n", incoming_msg);
            printf("ACTION: Robot detected at Floor 2! Dispatching elevator...\n");
            printf("=========================================\n");
            
            elevator_called = true; 
            state_timer = 0;        
        }
    } 
    // --- Handle Clear Signals ---
    else if (strcmp(incoming_msg, "FLOOR_2_CLEAR") == 0) {
        if (elevator_called) {
            printf("\nSTATUS: Floor 2 area is now clear.\n");
            elevator_called = false;
        }
    }
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

    // LOCK TO CHANNEL 1: Ensures the controller sits on the right frequency lane 
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE)); 

    // 4. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // 5. Register BOTH callbacks
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv)); 

    // 6. Register the Peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, receiver_mac, 6);
    peer_info.channel = 1; 
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        printf("Failed to add peer!\n");
        return;
    }

    printf("Elevator Control Unit Online. Waiting for wireless requests...\n");
    
    // 7. Simulated Travel Task Loop
    while (1) {
        if (elevator_called) {
            state_timer++;
            
            if (state_timer >= 5) {
                printf("\n[ELEVATOR] Car has arrived at Floor 2. Opening doors for the robot!\n\n");
                elevator_called = false; 
                state_timer = 0;         
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}