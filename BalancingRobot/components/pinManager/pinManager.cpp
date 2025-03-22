#include "pinManager.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *PM_TAG = "pinManager";

void PM_Configuration(void)
{
    ESP_LOGI(PM_TAG, "PM_Configuration... ");

    gpio_reset_pin(M1_DIAG);
    gpio_reset_pin(M2_DIAG);
    gpio_reset_pin(BMI160_INT1);
    gpio_reset_pin(BMI160_INT2);
    gpio_reset_pin(M_EN);
    gpio_reset_pin(M1_DIR);
    gpio_reset_pin(M1_STEP);
    gpio_reset_pin(BMI160_I2C_SDA);
    gpio_reset_pin(BMI160_I2C_SCL);
    gpio_reset_pin(M_MS2_AD1);
    gpio_reset_pin(M_SPREAD);
    gpio_reset_pin(M2_STEP);
    gpio_reset_pin(M2_DIR);
    gpio_reset_pin(SERVO1);
    gpio_reset_pin(SR04_ECHO);
    gpio_reset_pin(SR04_TRIGGER);
    gpio_reset_pin(M_MS1_AD0);
    gpio_reset_pin(ERROR_PIN);

    gpio_set_direction(ERROR_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(BMI160_INT1, GPIO_MODE_INPUT);
    gpio_set_direction(BMI160_INT2, GPIO_MODE_INPUT);

    gpio_set_direction(M_EN, GPIO_MODE_OUTPUT);
    gpio_set_direction(M_SPREAD, GPIO_MODE_OUTPUT);
    gpio_set_direction(M_MS1_AD0, GPIO_MODE_OUTPUT);
    gpio_set_direction(M_MS2_AD1, GPIO_MODE_OUTPUT);

    gpio_set_direction(M1_DIAG, GPIO_MODE_INPUT);
    gpio_set_direction(M1_DIR, GPIO_MODE_OUTPUT);
    gpio_set_direction(M1_STEP, GPIO_MODE_OUTPUT);

    gpio_set_direction(M2_DIAG, GPIO_MODE_INPUT);
    gpio_set_direction(M2_DIR, GPIO_MODE_OUTPUT);
    gpio_set_direction(M2_STEP, GPIO_MODE_OUTPUT);

    ESP_LOGI(PM_TAG, "PM_Configuration COMPLETE!");
}