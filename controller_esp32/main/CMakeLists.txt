idf_component_register(SRCS "bt.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_wifi esp_netif nvs_flash)

