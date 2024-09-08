idf_component_register(SRCS "main.c" "mqtt-client.c" "ota-api.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash
                    PRIV_REQUIRES UDPlogger esp_wifi esp32-homekit esp_driver_i2s esp_driver_gpio esp_timer esp32-ds18b20 app_update mqtt)
