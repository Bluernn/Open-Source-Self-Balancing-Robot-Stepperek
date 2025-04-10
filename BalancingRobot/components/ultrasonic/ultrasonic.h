/**
 * @file ultrasonic.h
 *
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and so on
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include <driver/gpio.h>

#define ESP_ERR_ULTRASONIC_PING         0x200
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202

/**
 * Device descriptor
 */
typedef struct
{
    gpio_num_t trigger_pin;
    gpio_num_t echo_pin;
} ultrasonic_sensor_t;

/**
 * Init ranging module
 * \param dev Pointer to the device descriptor
 */
void ultrasonic_init(const ultrasonic_sensor_t *dev);

/**
 * Measure distance
 * \param dev Pointer to the device descriptor
 * \param max_distance Maximal distance to measure, centimeters
 * \return Distance in centimeters or ULTRASONIC_ERROR_xxx if error occured
 */
esp_err_t ultrasonic_measure_m(const ultrasonic_sensor_t *dev, uint32_t max_distance, float *distance);

#endif /* __ULTRASONIC_H__ */