#include <inttypes.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include <esp_timer.h>
#include "driver/i2c.h"
#include "cJSON.h"

#include "websocket_server.h"
#include "mdns.h"
#include "../components/pinManager/pinManager.h"
#include "../components/stepperMotorController/stepperMotorController.h"
#include "../components/webServerClient/webServerClient.h"
#include "../components/controllers/controllers.h"

static const char *TAG = "MAIN";
static const char *MDNS_HOSTNAME = "selfbalancingrobot";

MessageBufferHandle_t xMessageBufferToClient;

extern "C" {
	void app_main(void);
}

void bmi160(void *pvParameters);
void BMI_send_data(void *pvParameters);

#ifdef __cplusplus
extern "C" {
#endif
void start_mdns(void);
void start_i2c(void);
int ws_server_start(void);
#include "../components/wifi/wifi.h"
#include "../components/ultrasonic/ultrasonic.h"
#ifdef __cplusplus
}
#endif

void start_mdns(void)
{
	//initialize mDNS
	ESP_ERROR_CHECK( mdns_init() );
	//set mDNS hostname (required if you want to advertise services)
	ESP_ERROR_CHECK( mdns_hostname_set(MDNS_HOSTNAME) );
	ESP_LOGI(TAG, "mdns hostname set to: [%s]", MDNS_HOSTNAME);

	//initialize service
	ESP_ERROR_CHECK( mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0) );
}

void start_i2c(void) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)BMI160_I2C_SDA;
	conf.scl_io_num = (gpio_num_t)BMI160_I2C_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

extern volatile RobotMode currentMode;
extern volatile bool isStable;
extern volatile int robotEnable;
extern volatile PStructure Position;
extern volatile PIStructure Velocity;
extern volatile PDStructure Angle;
extern volatile double setPosition;
extern volatile double setVelocity;
extern volatile double setVelocityTurnFactor;
extern volatile bool isStation;
extern volatile double roll;
//extern volatile float rawRoll;

// Define queue to share data between tasks
QueueHandle_t dataQueue;

// Data structure for storing IMU values
typedef struct {
    double p_ref, p, v_ref, v, O_ref, O, a;
} SensorData;

void IRAM_ATTR timerCallbackSD(void *arg) {
	if (!isStation) return;
	
    // Data preparation
	SensorData data = {
		.p_ref = Position.SetpointValue,
		.p = Position.MeasuredValue,
		.v_ref = Velocity.SetpointValue,
		.v = Velocity.MeasuredValue,
		//.O_ref = rawRoll,
		.O_ref = Position.SetpointValue,
		.O = roll,
		.a = Angle.OutputValue
	};
	
    // Sending data to the queue
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(dataQueue, &data, &xHigherPriorityTaskWoken) != pdPASS) {
		ESP_LOGE(TAG, "Queue send failed in ISR");
    }

    // Force context switching, if necessary
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void setupTimerSD() {
	ESP_LOGI(TAG, "Starting setupTimerSD ...");
    const esp_timer_create_args_t timer_argsSD = {
        .callback = &timerCallbackSD,
        .arg = NULL,               
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sensor_timer"
    };

    esp_timer_handle_t timer_handleSD;
    ESP_ERROR_CHECK(esp_timer_create(&timer_argsSD, &timer_handleSD));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handleSD, 10000));

	ESP_LOGI(TAG, "setupTimerSD timer initialized.");
}

void sequenceTimerCallback(void *arg) {
    static int state = 0;

    // Check if the control is active
    if (!isStable && robotEnable == 0) {
        Velocity.SetpointValue = 0.0;
		state = 0;
        return;
    }

	if ( currentMode == QUADRANT_SEQUENCE_MODE )
	{
		switch (state) {
			case 0: 
				setVelocityTurnFactor = 0;
				state = 1;
				break; 
			case 1:
				setVelocityTurnFactor = 0.5;
				state = 0;
			break; 
		}
	}
    // else if (currentMode == VELOCITY_MODE) {  // FOR TESTING THE VELOCITY CONTROLLER
    //     switch (state) {
    //         case 0:
    //             Velocity.SetpointValue = 0.0; // Stop
    //             state = 1;
    //             break;
    //         case 1:
    //             Velocity.SetpointValue = setVelocity; // Move backward
    //             state = 2;
    //             break;
	// 		case 2:
	// 		    state = 3;
    //             break;
    //         case 3:
    //             Velocity.SetpointValue = 0.0; // Stop
    //             state = 4;
    //             break;
    //         case 4:
    //             Velocity.SetpointValue = -setVelocity; // Move forward
    //             state = 5;
    //             break;
	// 		case 5:
	// 		    state = 0;
    //             break;
    //     }
    // }
	// else if (currentMode == POSITION_MODE) { // FOR TESTING THE POSITION CONTROLLER
    //     switch (state) {
    //         case 0:
    //             Position.SetpointValue = 0.0; // Stop
    //             state = 1;
    //             break;
	// 		case 1:
	// 			state = 2;
	// 			break;
    //         case 2:
	// 			Position.SetpointValue = setPosition; // Move backward
    //             state = 3;
    //             break;
	// 		case 3:
	// 		    state = 4;
    //             break;
    //         case 4:
	// 			Position.SetpointValue = 0.0; // Stop
    //             state = 5;
    //             break;
	// 		case 5:
	// 			state = 6;
	// 			break;
    //         case 6:
	// 			Position.SetpointValue = -setPosition; // Move forward
    //             state = 7;
    //             break;
	// 		case 7:
	// 		    state = 0;
    //             break;
    //     }
    // }
	else 
	{
        Velocity.SetpointValue = 0.0;
		//Position.SetpointValue = 0.0;
    }
}

void setupSequenceTimer() {
	ESP_LOGI(TAG, "Starting setupSequenceTimer...");
    
    const esp_timer_create_args_t sequence_timer_args = {
        .callback = &sequenceTimerCallback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sequence_timer"
    };

	esp_timer_handle_t sequence_timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&sequence_timer_args, &sequence_timer_handle));
    
    ESP_ERROR_CHECK(esp_timer_start_periodic(sequence_timer_handle, 1000000)); // sec

	ESP_LOGI(TAG, "setupSequenceTimer initialized.");
}

#define MAX_DISTANCE_M 5 // 5m max
volatile float ultrasonicDistance = 0.0f;
ultrasonic_sensor_t ultrasonicSensor = {
	.trigger_pin = SR04_TRIGGER,
	.echo_pin = SR04_ECHO
};

void UltrasonicTask(void *pvParameters)
{
    while(1) {
		float distance;
		esp_err_t res = ultrasonic_measure_m(&ultrasonicSensor, MAX_DISTANCE_M, &distance);

		if ( res == ESP_OK )
			ultrasonicDistance = distance;

		if (currentMode == SENSOR_MODE)
			vTaskDelay(pdMS_TO_TICKS(100));
		else
			vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task responsible for sending data every 2 ms
void SendDataTask(void *pvParameters) {
    SensorData data;

    while (1) {
        if (dataQueue != NULL)
        {
            // Try receiving data from the queue
            if (xQueueReceive(dataQueue, &data, portMAX_DELAY) == pdPASS) {
                // Prepare JSON data
                cJSON *request = cJSON_CreateObject();
                cJSON_AddStringToObject(request, "id", "data-request");
                cJSON_AddNumberToObject(request, "p_ref", data.p_ref);
                cJSON_AddNumberToObject(request, "p", data.p);
                cJSON_AddNumberToObject(request, "v_ref", data.v_ref);
                cJSON_AddNumberToObject(request, "v", data.v);
                cJSON_AddNumberToObject(request, "O_ref", data.O_ref);
                cJSON_AddNumberToObject(request, "O", data.O);
                cJSON_AddNumberToObject(request, "a", data.a*0.001);

                // Convert JSON to string
                char *my_json_string = cJSON_Print(request);

                // Send data over the message buffer
                size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
                if (xBytesSent != strlen(my_json_string)) {
                    ESP_LOGE(TAG, "Failed to send message buffer");
                }

                // Free resources
                cJSON_Delete(request);
                cJSON_free(my_json_string);
            }        
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
	// Create a queue to hold 10 SensorData element
	dataQueue = xQueueCreate(64, sizeof(SensorData));

	// Initialize ESP Pins
	PM_Configuration();

	// Initialize StepperMotors
	SMC_StepperInit();

	// Initialize WiFi
	setup_wifi();

	// Initialize mDNS
	start_mdns();

	// Initialize i2c
	start_i2c();

	// Initialize ultrasonic
	ultrasonic_init(&ultrasonicSensor);

	// Create Message Buffer
	xMessageBufferToClient = xMessageBufferCreate(1024*4);
	configASSERT( xMessageBufferToClient );

	// Start web socket server
	ws_server_start();

	// Start web server
	xTaskCreate(&server_task, "SERVER", 1024*2, NULL, 5, NULL);

	// Start web client
	xTaskCreate(&client_task, "CLIENT", 1024*3, (void *)0x011, 5, NULL);

	// Start imu task
	xTaskCreate(&bmi160, "IMU", 1024*8, NULL, 5, NULL);

	xTaskCreate(StabilityMonitorTask, "StabilityMonitor", 2048, NULL, 5, NULL);

	xTaskCreate(&SMC_StepperControl, "StepperControl", 1024*8, NULL, 5, NULL);

	xTaskCreate(SendDataTask, "Send Data Task", 1024*4, NULL, 5, NULL);

	xTaskCreate(UltrasonicTask, "Ultrasonic Task", 1024*2, NULL, 5, NULL);

	setupTimerSD();
	setupSequenceTimer();
}
