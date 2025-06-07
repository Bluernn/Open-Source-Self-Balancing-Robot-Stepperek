#include "controllers.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

volatile PDStructure Angle;
volatile PIStructure Velocity;
volatile PStructure Position;
volatile RobotMode currentMode = BALANCE_MODE;

static const char *C_TAG = "controllers";

void C_PCalc(void) 
{
    // Calculate the error
    double error = Position.SetpointValue - Position.MeasuredValue;

    // Calculate the output value
    Position.OutputValue = Position.Kp * error;
}

void C_PICalc(void) 
{
    // Protection against excessive speed
    if (Velocity.SetpointValue > Velocity.MinMaxInputValue) {
        Velocity.SetpointValue = Velocity.MinMaxInputValue;
    } else if (Velocity.SetpointValue < -Velocity.MinMaxInputValue) {
        Velocity.SetpointValue = (-1.0f) * Velocity.MinMaxInputValue;
    }

    // Calculate the error
    double error = Velocity.SetpointValue - Velocity.MeasuredValue;

    // Calculate the integral value 1) without Anti-windup 2) with Anti-windup
    //1)float integralValue = (error + Velocity.PreviousErrorValue) * Tp / 2 + Velocity.PreviousIntegralValue;
    double integralValue = Velocity.PreviousIntegralValue + (error + Velocity.PreviousErrorValue - Velocity.PreviousSaturationValue) * Tp / 2;
    
    // Calculate the output value
    Velocity.OutputValue = ( Velocity.Kp * error ) + ( Velocity.Ki * integralValue );

    // Apply output limitations (saturation)
    double u_sat;
    if (Velocity.OutputValue > Velocity.MinMaxOutputValue) {
        u_sat = Velocity.MinMaxOutputValue;
    } else if (Velocity.OutputValue < -Velocity.MinMaxOutputValue) {
        u_sat = (-1.0f) * Velocity.MinMaxOutputValue;
    } else {
        u_sat = Velocity.OutputValue;
    } 

    // Anti-windup
    double sat_error = Velocity.OutputValue - u_sat;
    Velocity.PreviousSaturationValue = Velocity.Kawu * sat_error;

    Velocity.OutputValue = u_sat;

    // Update previous values
    Velocity.PreviousErrorValue = error;
    Velocity.PreviousIntegralValue = integralValue;
}

void C_PDCalc(void) 
{
    // Calculate the error
    double error = Angle.SetpointValue - Angle.MeasuredValue;

    // Calculate the derivative value
    double derivativeValue = (-Angle.MeasuredValue + Angle.PreviousMeasuredValue) / Tp;

    // Calculate the output value
    Angle.OutputValue = ( Angle.Kp * error ) + ( Angle.Kd * derivativeValue );

    // Apply output limitations
    if (Angle.OutputValue > Angle.MinMaxOutputValue) {
        Angle.OutputValue = Angle.MinMaxOutputValue;
    } else if (Angle.OutputValue < -Angle.MinMaxOutputValue) {
        Angle.OutputValue = (-1.0f) * Angle.MinMaxOutputValue;
    }

    // Update previous values
    Angle.PreviousMeasuredValue = Angle.MeasuredValue;
}

void C_init(void)
{
    ESP_LOGI(C_TAG,"C_init... ");

    // Position controller
    Position.Kp = KP_P;
    Position.OutputValue = 0;
    Position.MinMaxOutputValue = 0;
    Position.SetpointValue = 0;

    // Linear Velocity controller
    Velocity.Kp = KP_PI;
    Velocity.Ki = KI_PI;
    Velocity.Kawu = KAWU_PI;
    Velocity.OutputValue = 0;
    Velocity.MinMaxOutputValue = 5;
    Velocity.MinMaxInputValue = 0.4; // 1
    Velocity.PreviousIntegralValue = 0;
    Velocity.PreviousErrorValue = 0;
    Velocity.PreviousSaturationValue = 0;
    Velocity.PreviousSetpointValue = 0;

    // Tilt Angle controller
    Angle.Kp = KP_PD;
    Angle.Kd = KD_PD;
    Angle.OutputValue = 0;
    Angle.MinMaxOutputValue = 18000;
    Angle.PreviousMeasuredValue = 0;

    ESP_LOGI(C_TAG, "C_init COMPLETE!");
}