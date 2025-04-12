#ifndef PIDCONTROLLER_H
#define	PIDCONTROLLER_H

// Sampling period
#define Tp 0.001

// Position controller
#define KP_P 2

// Velocity controller
#define KP_PI 40
#define KI_PI 30
#define KAWU_PI 0.04

// Angle controller
#define KP_PD 1
#define KD_PD 0.15

enum RobotMode {
    BALANCE_MODE,
    QUADRANT_SEQUENCE_MODE,
    POSITION_MODE,
    SENSOR_MODE,
    CTRL_MODE,
    REMOTE_MODE,
    TEST_MODE_POSITION,
    TEST_MODE_VELOCITY
};

typedef struct PIStructure {
    volatile double SetpointValue;
    volatile double PreviousSetpointValue;
    volatile double MeasuredValue;
    volatile float Kp;
    volatile float Ki;
    volatile float Kawu;
    volatile double OutputValue;
    volatile float MinMaxInputValue;
    volatile float MinMaxOutputValue;
    volatile double PreviousIntegralValue;
    volatile double PreviousErrorValue;
    volatile double PreviousSaturationValue;
} PIStructure;

void C_PICalc(void);

typedef struct PDStructure {
    volatile double SetpointValue;
    volatile double MeasuredValue;
    volatile float Kp;
    volatile float Kd;
    volatile double OutputValue;
    volatile float MinMaxOutputValue;
    volatile double PreviousMeasuredValue;
} PDStructure;

void C_PDCalc(void);

typedef struct PStructure {
    volatile double SetpointValue;
    volatile double MeasuredValue;
    volatile float Kp;
    volatile double OutputValue;
    volatile float MinMaxOutputValue;
} PStructure;

void C_PCalc(void);

void C_init(void);

#endif
