idf_component_register(SRCS "reed_switch.c" "main.c" "sensor.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_wifi esp_netif esp_now nimble)
 