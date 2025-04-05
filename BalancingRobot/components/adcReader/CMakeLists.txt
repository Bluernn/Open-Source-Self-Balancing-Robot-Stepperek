idf_component_register(
    SRCS "adcReader.cpp"
    INCLUDE_DIRS .
    REQUIRES "freertos" "driver" "esp_adc"
)
