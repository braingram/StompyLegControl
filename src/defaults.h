#ifndef DEFAULTS_H
#define DEFAULTS_H

#define ADC_N_AVG 8
#define ADC_RES 16

// === PWM values ===
// assumes PWM_RES of 13
#define HIP_EXTEND_PWM_MIN 2457  // 30%
#define HIP_RETRACT_PWM_MIN 2457  // 30%
#define HIP_PWM_MAX 4096  // 50%
//#define HIP_ADC_MIN 5952  // FR
//#define HIP_ADC_MAX 46208
#define HIP_EXTEND_PWM_MIN 0  // 30%
#define HIP_RETRACT_PWM_MIN 0  // 30%
#define HIP_PWM_MAX 4096  // 50%
#define HIP_ADC_MIN 947  // motorized pot
#define HIP_ADC_MAX 64017

#define THIGH_EXTEND_PWM_MIN 2457
#define THIGH_RETRACT_PWM_MIN 2457
#define THIGH_PWM_MAX 8192
#define THIGH_ADC_MIN 2176
#define THIGH_ADC_MAX 58688

#define KNEE_EXTEND_PWM_MIN 2457
#define KNEE_RETRACT_PWM_MIN 2457
#define KNEE_PWM_MAX 8192
#define KNEE_ADC_MIN 9472
#define KNEE_ADC_MAX 59776


// === PID values ===
//#define HIP_P 2.0
//#define HIP_I 0.0
#define HIP_P 5.0
#define HIP_I 2.0
#define HIP_D 0.0
#define HIP_PID_MIN -65535
#define HIP_PID_MAX 65535

#define THIGH_P 24.0
//#define THIGH_I 1.0  // TODO scale by 1000?
#define THIGH_I 0.0
#define THIGH_D 0.0
#define THIGH_PID_MIN -65535
#define THIGH_PID_MAX 65535

#define KNEE_P 17.0
//#define KNEE_I 1.0  // TODO scale by 1000?
#define KNEE_I 0.0
#define KNEE_D 0.0
#define KNEE_PID_MIN -65535
#define KNEE_PID_MAX 65535

#endif
