idf_component_register(
    SRCS "stepperMotorController.cpp"
    INCLUDE_DIRS .
    REQUIRES "freertos" "driver" "controllers" "esp_timer"
)
