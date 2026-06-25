#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "sensor.h"       // Keeps your optical sensor capability available
#include "reed_switch.h"  // Includes your magnetic switch functions

// Hardcoded MAC address of your Controller (#1) board
uint8_t controller_mac[] = {0x30, 0x76, 0xF5, 0xF8, 0x4D, 0x7C}; 

// Callback function that triggers when data is received from the controller
void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    printf("Message received from Controller!\n");
    printf("Data length: %d bytes\n", len);
    printf("Data content: %.*s\n", len, data);
    printf("---------------------------\n");
}

// NEW Callback: Tells us exactly if the controller physically acknowledged the packet
void on_data_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        printf("[ESP-NOW] Send Success! Controller received it.\n");
    } else {
        printf("[ESP-NOW] Send FAIL. Controller is offline or MAC is wrong.\n");
    }
}

void app_main(void) {
    // 1. Initialize NVS (Required for Wi-Fi configurations)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize Wi-Fi stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // FORCE RADIO TO CHANNEL 1: Matches the controller frequency channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // 3. GET AND PRINT THE MAC ADDRESS
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    printf("\n======================================================\n");
    printf("MY RECEIVER MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\n", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("======================================================\n\n");

    // 4. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // 5. Register the callbacks
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent)); 

    // 6. Register the Controller as an ESP-NOW Peer so we can send data to it
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, controller_mac, 6);
    peer_info.channel = 1; // Locked to Channel 1
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        printf("Failed to add controller peer!\n");
        return;
    }

    // 7. Initialize BOTH components
    init_proximity_sensor(); // Sets up D15
    init_reed_switch();      // Sets up D4

    char message[32];
    bool magnet_was_present = false;

    // --- NEW: BOOTUP HANDSHAKE PACKET ---
    printf("[SYSTEM] Sending bootup connection packet to Controller...\n");
    snprintf(message, sizeof(message), "BOARD_2_CONNECTED");
    esp_now_send(controller_mac, (uint8_t *)message, strlen(message));
    // ------------------------------------

    printf("Receiver is ready, listening, and monitoring D4 for Magnet...\n");

    // 8. Infinite loop checking the reed switch state
    while (1) {
        bool magnet_present = is_magnet_present(); 

        if (magnet_present && !magnet_was_present) {
            printf("[MAGNET] Switch Closed! Sending call to Elevator...\n");
            
            snprintf(message, sizeof(message), "CALL_ELEVATOR_FLOOR_2");
            esp_now_send(controller_mac, (uint8_t *)message, strlen(message));
            
            magnet_was_present = true; 
        } 
        else if (!magnet_present && magnet_was_present) {
            printf("[MAGNET] Switch Opened! Clearing status...\n");
            
            snprintf(message, sizeof(message), "FLOOR_2_CLEAR");
            esp_now_send(controller_mac, (uint8_t *)message, strlen(message));
            
            magnet_was_present = false;
        }

        // Poll every 500ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}