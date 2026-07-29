idf_component_register(SRCS "reed_switch.c" "main.c" "sensor.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_wifi esp_netif nvs_flash bt)
 