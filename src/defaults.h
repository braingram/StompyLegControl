#ifndef DEFAULTS_H
#define DEFAULTS_H

#define ADC_N_AVG 16
#define ADC_RES 16
#define ADC_CONV_SPEED ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS
#define ADC_SAMP_SPEED ADC_SAMPLING_SPEED::HIGH_SPEED

// === PWM values ===
// assumes PWM_RES of 13
//#define HIP_EXTEND_PWM_MIN 2457  // 30%
//#define HIP_EXTEND_PWM_MAX 4096  // 50%
//#define HIP_RETRACT_PWM_MIN 2457  // 30%
//#define HIP_RETRACT_PWM_MAX 4096  // 50%
//#define HIP_ADC_MIN 5952  // FR
//#define HIP_ADC_MAX 46208
#define HIP_ADC_MIN 6155  // fr 170904
#define HIP_ADC_MAX 51979  // fr 170904

// --- motorized pot ---
//#define HIP_ADC_MIN 0
//#define HIP_ADC_MAX 65535
//#define HIP_EXTEND_PWM_MIN 0  // 0%
#define HIP_EXTEND_PWM_MIN 1638  // 20%
//#define HIP_EXTEND_PWM_MAX 4096  // 50%
#define HIP_EXTEND_PWM_MAX 8192  // 50%
//#define HIP_RETRACT_PWM_MIN 0  // 30%
#define HIP_RETRACT_PWM_MIN 1638  // 20%
//#define HIP_RETRACT_PWM_MAX 4096  // 50%
#define HIP_RETRACT_PWM_MAX 8192  // 50%

//#define THIGH_EXTEND_PWM_MIN 2457
//#define THIGH_EXTEND_PWM_MAX 8192
//#define THIGH_RETRACT_PWM_MIN 2457
//#define THIGH_RETRACT_PWM_MAX 8192
//#define THIGH_ADC_MIN 2176
//#define THIGH_ADC_MAX 58688
#define THIGH_ADC_MIN 2218  // fr 170904
#define THIGH_ADC_MAX 60632  // fr 170904
//
// --- motorized pot ---
//#define THIGH_ADC_MIN 0  // motorized pot
//#define THIGH_ADC_MAX 65535
//#define THIGH_EXTEND_PWM_MIN 0
#define THIGH_EXTEND_PWM_MIN 1638  // 20%
//#define THIGH_EXTEND_PWM_MAX 4096
#define THIGH_EXTEND_PWM_MAX 8192
//#define THIGH_RETRACT_PWM_MIN 0
#define THIGH_RETRACT_PWM_MIN 1638  // 20%
//#define THIGH_RETRACT_PWM_MAX 4096
#define THIGH_RETRACT_PWM_MAX 8192


//#define KNEE_EXTEND_PWM_MIN 2457
//#define KNEE_EXTEND_PWM_MAX 8192
//#define KNEE_RETRACT_PWM_MIN 2457
//#define KNEE_RETRACT_PWM_MAX 8192
//#define KNEE_ADC_MIN 9472
//#define KNEE_ADC_MAX 59776
#define KNEE_ADC_MIN 3417  // fr 170904
#define KNEE_ADC_MAX 54144  // fr 170904
//
// --- motorized pot ---
//#define KNEE_ADC_MIN 0
//#define KNEE_ADC_MAX 65535
//#define KNEE_EXTEND_PWM_MIN 0
#define KNEE_EXTEND_PWM_MIN 1638  // 20%
//#define KNEE_EXTEND_PWM_MAX 4096
#define KNEE_EXTEND_PWM_MAX 8192
//#define KNEE_RETRACT_PWM_MIN 0
#define KNEE_RETRACT_PWM_MIN 1638  // 20%
//#define KNEE_RETRACT_PWM_MAX 4096
#define KNEE_RETRACT_PWM_MAX 8192


// === PID values ===
//#define HIP_P 2.0
//#define HIP_I 0.0
// -- for linear pot --
//#define HIP_P 30.0
//#define HIP_I 20.0
#define HIP_P 5.0
#define HIP_I 1.0
#define HIP_D 0.0
//#define HIP_PID_MIN -65535
//#define HIP_PID_MAX 65535
#define HIP_PID_MIN -8192
#define HIP_PID_MAX 8192

//#define THIGH_P 24.0
#define THIGH_P 5.0
//#define THIGH_I 1.0  // TODO scale by 1000?
#define THIGH_I 1.0
#define THIGH_D 0.0
//#define THIGH_PID_MIN -65535
//#define THIGH_PID_MAX 65535
#define THIGH_PID_MIN -8192
#define THIGH_PID_MAX 8192

//#define KNEE_P 17.0
#define KNEE_P 5.0
//#define KNEE_I 1.0  // TODO scale by 1000?
#define KNEE_I 1.0
#define KNEE_D 0.0
//#define KNEE_PID_MIN -65535
//#define KNEE_PID_MAX 65535
#define KNEE_PID_MIN -8192
#define KNEE_PID_MAX 8192

#endif
