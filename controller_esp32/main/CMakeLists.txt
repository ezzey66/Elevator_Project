idf_component_register(SRCS "bt.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_wifi esp_netif esp_now)

