idf_component_register(
    SRCS "wifi.c"
    INCLUDE_DIRS .
    REQUIRES "freertos" "driver" "esp_wifi" "nvs_flash" "esp_http_server" "spiffs"
)
