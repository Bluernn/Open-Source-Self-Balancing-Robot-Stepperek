/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/i2c.h"


extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// bmi160 stuff
#include "bmi160.h"
#include "bmi160_defs.h"
#define I2C_NUM I2C_NUM_0

// Source: https://github.com/arduino-libraries/MadgwickAHRS
#include "MadgwickAHRS.h"


#include "../components/controllers/controllers.h"

//volatile float rawRoll = 0;
volatile double roll;
volatile bool initialized = false;

#define OFFSET 81
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

Madgwick madgwick;

/* IMU Data */
struct bmi160_dev sensor;

// Accel & Gyro scale factor
double accel_sensitivity;
double gyro_sensitivity;

/** Select an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 */
void SelectRegister(uint8_t devAddr, uint8_t regAddr){
	ESP_LOGD(__FUNCTION__, "devAddr=0x%x regAddr=0x%x", devAddr, regAddr);
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
}


/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t user_i2c_read(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t length) {
	i2c_cmd_handle_t cmd;
	SelectRegister(devAddr, regAddr);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, 1));

	if(length>1)
		ESP_ERROR_CHECK(i2c_master_read(cmd, data, length-1, I2C_MASTER_ACK));

	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+length-1, I2C_MASTER_NACK));

	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "regAddr=0x%x length=%d", regAddr, length);
	for (int i=0;i<length;i++) {
		ESP_LOGI(__FUNCTION__, "data[%d]=0x%x", i, data[i]);
	}
#endif

	return 0;
}


/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param length Number of bytes to write
 * @param data Array of bytes to write
 * @return Status of operation (true = success)
 */
int8_t user_i2c_write(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint16_t length) {
#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "regAddr=0x%x length=%d", regAddr, length);
	for (int i=0;i<length;i++) {
		ESP_LOGI(__FUNCTION__, "data[%d]=0x%x", i, data[i]);
	}
#endif

	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	if(length>1)
		ESP_ERROR_CHECK(i2c_master_write(cmd, data, length-1, 0));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length-1], 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

	return 0;
}

void user_delay_ms(uint32_t period) {
#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "period=%" PRIu32, period);
#endif
	esp_rom_delay_us(period*1000);
}

// Get time in seconds since boot
// Compatible with ROS's time.toSec() function
double TimeToSec() {
	int64_t _time = esp_timer_get_time(); // Get time in microseconds since boot
	double __time = (double)_time / 1000000;
	return __time;
}

void bmi160(void *pvParameters)
{
    //I2C address is 0x68 (if SDO-pin is gnd) or 0x69 (if SDO-pin is vddio).
    //sensor.id = BMI160_I2C_ADDR;
    sensor.id = CONFIG_I2C_ADDR;
    sensor.intf = BMI160_I2C_INTF;
    sensor.read = user_i2c_read;
    sensor.write = user_i2c_write;
    sensor.delay_ms = user_delay_ms;

    int8_t ret = bmi160_init(&sensor);
    if (ret == BMI160_OK) {
        ESP_LOGI(TAG, "BMI160 initialization success !");
        ESP_LOGI(TAG, "Chip ID 0x%X", sensor.chip_id);
    } else {
        ESP_LOGE(TAG, "BMI160 initialization fail %d", ret);
        vTaskDelete(NULL);
    }

    // Config Accel
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_400HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // -2 --> +2[g]
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_RES_AVG8;
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    accel_sensitivity = 16384.0; // g

    // Config Gyro
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_400HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS; // -250 --> +250[Deg/Sec]
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
    gyro_sensitivity = 131.2; // Deg/Sec

    ret = bmi160_set_sens_conf(&sensor);
    if (ret != BMI160_OK) {
        ESP_LOGE(TAG, "BMI160 set_sens_conf fail %d", ret);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "bmi160_set_sens_conf");

    double last_time_ = TimeToSec();

	TickType_t xLastWakeTimeBMI = xTaskGetTickCount(); // Initial task startup time
   
    while(1) {
        struct bmi160_sensor_data accel;
        struct bmi160_sensor_data gyro;
        int8_t ret = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
        if (ret != BMI160_OK) {
            ESP_LOGE(TAG, "BMI160 get_sensor_data fail %d", ret);
            vTaskDelete(NULL);
        }

        // Calculate time delta
        double dt = (TimeToSec() - last_time_);
        last_time_ = TimeToSec();

        // Convert raw data to meaningful units
        double ax = (double)accel.x / accel_sensitivity;
        double ay = (double)accel.y / accel_sensitivity;
        double az = (double)accel.z / accel_sensitivity; 
        double gx = (double)gyro.x / gyro_sensitivity;
        double gy = (double)gyro.y / gyro_sensitivity;
        double gz = (double)gyro.z / gyro_sensitivity;

        // Calculate angles using the Madgwick filter
        madgwick.updateIMU(gx, gy, gz, ax, ay, az, dt);
        roll = madgwick.getRoll() + OFFSET;
        //rawRoll = ( 57.29578f * (atan2f(ay,az)) + OFFSET);

        // Delay for 2.5 ms
        vTaskDelayUntil(&xLastWakeTimeBMI, pdMS_TO_TICKS(2.5));

    } // end while

    vTaskDelete(NULL);
}