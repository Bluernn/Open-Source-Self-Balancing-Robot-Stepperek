#include "stepperMotorController.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "../pinManager/pinManager.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <esp_timer.h>

#include "../controllers/controllers.h"

#define MOTORS_ENABLE   gpio_set_level(M_EN, 0);
#define MOTORS_DISABLE  gpio_set_level(M_EN, 1);

#define DIR_1_FORWARD   gpio_set_level(M1_DIR, 0);
#define DIR_1_BACKWARD  gpio_set_level(M1_DIR, 1);
#define DIR_2_FORWARD   gpio_set_level(M2_DIR, 1);
#define DIR_2_BACKWARD  gpio_set_level(M2_DIR, 0)

//200 steps/rev motor with microstepping 1/8 = 1600
#define STEPS_NUMBER_PER_ROTATION 6400
#define WHEEL_RADIUS 0.0335 // m
#define TWO_PI 6.283185307179586476925286766559

#define STABILITY_CHECK_TIME_MS 3000 // 3 seconds to check if the robot has returned to a stable position
#define STABILITY_TOLERANCE 5 // Tolerance range in degrees for stability

#define MAX_PWM_FREQ 70000

extern volatile double setVelocity;
extern volatile double setVelocityTurnFactor;
extern volatile double setPosition;
extern volatile float ultrasonicDistance;
volatile int robotEnable = 0; // WEB ON 
volatile bool robotStable = false;
volatile double angularVelocity = 0, acceleration = 0;
volatile bool controlEnable = false;
volatile bool isStable = false; // Flag to enable motor control after stability check
volatile TickType_t stabilityCheckStartTime = 0;

volatile uint16_t m1Freq = 1000;
volatile uint16_t m2Freq = 1000;
volatile uint8_t PWMDutyMotor1 = 0;
volatile uint8_t PWMDutyMotor2 = 0;

extern volatile double roll;
extern volatile RobotMode currentMode;

extern volatile PDStructure Angle;
extern volatile PIStructure Velocity;
extern volatile PStructure Position;

static const char *SMC_TAG = "stepperMotorController";

/* 0 - 1/8, 
1 - 1/16, 
2 - 1/32, 
3 - 1/64 */
static void SMC_MicrostepResolutionConfiguration(uint8_t config)
{
    ESP_LOGI(SMC_TAG,"SMC_MicrostepResolutionConfiguration... ");

    switch (config)
    {
    case 1: // 1/16
        gpio_set_level(M_MS1_AD0, 1);
        gpio_set_level(M_MS2_AD1, 1);
        break;
    case 2: // 1/32
        gpio_set_level(M_MS1_AD0, 1);
        gpio_set_level(M_MS2_AD1, 0);
        break;
    case 3: // 1/64
        gpio_set_level(M_MS1_AD0, 0);
        gpio_set_level(M_MS2_AD1, 1);
        break;
    default: // 1/8
        gpio_set_level(M_MS1_AD0, 0);
        gpio_set_level(M_MS2_AD1, 0);
        break;
    }

    ESP_LOGI(SMC_TAG, "SMC_MicrostepResolutionConfiguration COMPLETE!");
}

static void SMC_PWMConfig(void)
{
    ESP_LOGI(SMC_TAG,"SMC_PWMConfig... ");

    // PWM channel configuration for the first STEP
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = m1Freq,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = M1_STEP,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = PWMDutyMotor1,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    // PWM channel configuration for the second STEP
    ledc_timer_config_t ledc_timer2 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = m2Freq,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer2));

    ledc_channel_config_t ledc_channel2 = {
        .gpio_num = M2_STEP,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .duty = PWMDutyMotor2,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));

    ESP_LOGI(SMC_TAG, "SMC_PWMConfig COMPLETE!");
}

void SMC_StepperInit(void)
{   
    ESP_LOGI(SMC_TAG,"SMC_StepperInit... ");

    SMC_MicrostepResolutionConfiguration(2);
    SMC_PWMConfig();

    MOTORS_DISABLE;

    DIR_1_FORWARD;
    DIR_2_FORWARD;

    C_init();

    ESP_LOGI(SMC_TAG, "SMC_StepperInit COMPLETE!");
}

static void SMC_SetMotorsPWMDuty(uint8_t motor, uint16_t tmpDuty)
{
    // Select channel based on motor number
    ledc_channel_t channel = (motor == 1) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;

    // Get the current PWM for the selected channel
    uint32_t currentDuty = ledc_get_duty(LEDC_HIGH_SPEED_MODE, channel);

    // Update only if PWM is different from tmpDuty
    if (currentDuty != tmpDuty)
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, tmpDuty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);

        // Update the value of the stored PWM for a given motor
        if (motor == 1) PWMDutyMotor1 = tmpDuty;
        else PWMDutyMotor2 = tmpDuty;
    }
}

void StabilityMonitorTask(void *param) {
    while (true) {
        if (controlEnable == false)
        {
            if (roll >= STABILITY_TOLERANCE || roll <= -STABILITY_TOLERANCE) {
                // Robot is not stable, reset the time
                stabilityCheckStartTime = xTaskGetTickCount();
                isStable = false;
            } else {
                // Check if the robot was stable for a set period of time
                if ((xTaskGetTickCount() - stabilityCheckStartTime) >= pdMS_TO_TICKS(STABILITY_CHECK_TIME_MS)) {
                    isStable = true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void SMC_StepperControl(void *param)
{
    DIR_1_BACKWARD;
    DIR_2_BACKWARD;

    while(1) {
        if(isStable && (roll < 45 && roll > -45) && (robotEnable == 1))
        {
            controlEnable = true;
            MOTORS_ENABLE;
            
            /* P Position controller */
            if (currentMode == POSITION_MODE || currentMode == BALANCE_MODE || 
                currentMode == SENSOR_MODE || currentMode == CTRL_MODE || 
                currentMode == TEST_MODE) {
                setVelocityTurnFactor = 0;
            }

            if ( currentMode == SENSOR_MODE )
            {
                Position.MeasuredValue = ultrasonicDistance;

                if ( (ultrasonicDistance <= 1) && (ultrasonicDistance >= 0.1) )
                    Position.SetpointValue = setPosition; 
                else
                    Position.SetpointValue = ultrasonicDistance; // Protection
            }
            else 
                Position.MeasuredValue += Tp * ( (angularVelocity * TWO_PI * WHEEL_RADIUS) / STEPS_NUMBER_PER_ROTATION );

            C_PCalc();

            // PI linear velocity controller
            if (currentMode == BALANCE_MODE) {
                Velocity.SetpointValue = 0; // Tryb domyślny po uruchomieniu
            } else if (currentMode == REMOTE_MODE || currentMode == QUADRANT_SEQUENCE_MODE) {
                Velocity.SetpointValue = setVelocity;
            } else if (currentMode == SENSOR_MODE || currentMode == POSITION_MODE) {
                Velocity.SetpointValue = Position.OutputValue;
            }

            Velocity.MeasuredValue = (angularVelocity * TWO_PI * WHEEL_RADIUS) / STEPS_NUMBER_PER_ROTATION;
            C_PICalc();
            
            /* PD Tilt angle controller */
            Angle.SetpointValue = Velocity.OutputValue;
            Angle.MeasuredValue = roll;
            C_PDCalc();
            
            acceleration = (-1.0f) * Angle.OutputValue * STEPS_NUMBER_PER_ROTATION;
            angularVelocity += acceleration * Tp;

            if(angularVelocity < 0)
            {
                DIR_1_FORWARD;
                DIR_2_FORWARD;
            }
            else
            {
                DIR_1_BACKWARD;
                DIR_2_BACKWARD;
            }
            
            int absVel = (int) angularVelocity;
            if(absVel < 0) absVel *= (-1);

            // For the turning function, set the PWM of each wheel separately
            int absLeftWheelPWM = absVel * (1 - setVelocityTurnFactor);
            int absRightWheelPWM = absVel * (1 + setVelocityTurnFactor);

            // Limit the PWM frequency to the maximum value of MAX_PWM_FREQ
            absLeftWheelPWM = (absLeftWheelPWM <= MAX_PWM_FREQ) ? absLeftWheelPWM : MAX_PWM_FREQ;
            absRightWheelPWM = (absRightWheelPWM <= MAX_PWM_FREQ) ? absRightWheelPWM : MAX_PWM_FREQ;

            if(absLeftWheelPWM < 0) absLeftWheelPWM *= (-1);
            if(absRightWheelPWM < 0) absRightWheelPWM *= (-1);

            if (absLeftWheelPWM > 100) 
            {
                SMC_SetMotorsPWMDuty(1, 512);
                ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, absLeftWheelPWM);
            }
            else SMC_SetMotorsPWMDuty(1, 0);

            if (absRightWheelPWM > 100) 
            {
                SMC_SetMotorsPWMDuty(2, 512);
                ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, absRightWheelPWM);
            }
            else SMC_SetMotorsPWMDuty(2, 0);
        }
        else // Robot is tipped over, check for stability
        {
            MOTORS_DISABLE;
            Velocity.PreviousIntegralValue = 0;
            Velocity.PreviousErrorValue = 0;
            Velocity.PreviousSaturationValue = 0;
            Angle.PreviousMeasuredValue = 0;
            Position.MeasuredValue = 0;
            acceleration = 0;
            angularVelocity = 0;
            Velocity.Kp = KP_PI;
            Velocity.Ki = KI_PI;
            controlEnable = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Never reach here
    vTaskDelete(NULL);
}