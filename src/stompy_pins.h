/* Joystick */

#ifndef STOMPYPINS_H
#define STOMPYPINS_H

#define DEBUG_PIN_1 11
#define DEBUG_PIN_2 14

// TODO add options for test box
#ifdef OLD_BOARD
#define JOYSTICK_X_PIN		14
#define JOYSTICK_Y_PIN		15
#define JOYSTICK_Z_PIN		16
#define DEADMAN_PIN		0
#define ENABLE_PIN		1	/* Knee and thigh */
#define ENABLE_PIN_HIP		2
#else
#define JOYSTICK_X_PIN		31  // ADC1
#define JOYSTICK_Y_PIN		30  // ADC1
#define JOYSTICK_Z_PIN		29  // ADC1
#define DEADMAN_PIN		25
#define ENABLE_PIN		8	/* Knee and thigh */
#define ENABLE_PIN_HIP		2

#define PRESSURE_SENSOR_1	A17  // ADC1
#define PRESSURE_SENSOR_2	A16  // ADC1
#define PRESSURE_SENSOR_3	A15  // ADC1
#define PRESSURE_SENSOR_4	A12  // ADC0/ADC1
#define PRESSURE_SENSOR_MANIFOLD A13  // ADC0/ADC1
#endif

#define DISABLE_PIN 1

#define KNEE_SENSOR_PIN		15  // ADC0
#define THIGH_SENSOR_PIN	17  // ADC0/1
#define HIP_SENSOR_PIN		20  // ADC0
#define CALF_SENSOR_PIN	16  // ADC0/1

//These are mapped to the right front leg
#define THIGHPWM_DOWN_PIN	3
#define THIGHPWM_UP_PIN		4
#define KNEEPWM_RETRACT_PIN	5
#define KNEEPWM_EXTEND_PIN	6
#define HIPPWM_FORWARD_PIN	9
#define HIPPWM_REVERSE_PIN	10

// pwms mapped to the valve/cylinder positions
#define HIP_EXTEND_PIN	9
#define HIP_RETRACT_PIN	10
#define HIP_ENABLE_PIN 2
#define THIGH_EXTEND_PIN	3
#define THIGH_RETRACT_PIN 4
#define THIGH_ENABLE_PIN 8
#define KNEE_EXTEND_PIN 5
#define KNEE_RETRACT_PIN 6
#define KNEE_ENABLE_PIN 8

#define M1FB_PIN		21
#define M2FB_PIN		22
#define M1FB_HIP_PIN		23

#define STATUS_PIN 13

#endif
