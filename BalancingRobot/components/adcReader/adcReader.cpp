#include "adcReader.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *ADC_TAG = "adcReader";

#define DEFAULT_VREF    500        // Default reference voltage (mV)
#define ERROR_PIN GPIO_NUM_23
#define ADC_PIN ADC1_CHANNEL_0  // GPIO36 -> ADC1_CHANNEL_0

static esp_adc_cal_characteristics_t *adc_chars;
static uint32_t voltage_threshold = 240;  // Default threshold voltage value in mV (240 -> 2.65 V)

volatile bool lowVoltage = false;

void ADC_ReaderInit(void) {

    ESP_LOGI(ADC_TAG, "Starting ADC_ReaderInit...");

    // ADC1 initialization
    adc_chars = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    
    // ADC1 width configuration at 12-bit
    adc1_config_width(ADC_WIDTH_BIT_12); 
    
    // Suppression configuration for GPIO36 pin
    adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_0); 
    
    // Characteristics of the ADC1
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    ESP_LOGI(ADC_TAG, "ADC_ReaderInit initialized.");
}

// Task to monitor battery voltage
void ADC_TaskStart(void *pvParameter) {
    while (1) {
        int raw = adc1_get_raw(ADC_PIN);
        int battVoltage = esp_adc_cal_raw_to_voltage(raw, adc_chars);
        //ESP_LOGI(ADC_TAG, "Napięcie: %d mV", battVoltage);

        if (battVoltage < voltage_threshold) 
            lowVoltage = true;

        if (true == lowVoltage)
        {
            static bool led_state = false;
            gpio_set_level(ERROR_PIN, led_state);
            led_state = !led_state;
        } else {
            // Turn off the LED if the voltage is above the threshold
            gpio_set_level(ERROR_PIN, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
