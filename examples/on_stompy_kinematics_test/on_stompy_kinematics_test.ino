#include <ADC.h>
#include <RingBuffer.h>
#include <RingBufferDMA.h>
#include <ADC_Module.h>

#include "stompy_pins.h"
#include "geometry.h"
#include "sensors.h"
#include "transforms.h"
#include "point.h"
#include "kinematics.h"

#define PLOT_VALUES
#define PLOT_POTS
#define PLOT_ANGLES
#define PLOT_XYZ

ADC* adc = new ADC();

StringPot* hip_pot = new StringPot(
  HIP_SENSOR_PIN, adc, ADC_0,
  5952, 46208,  // 372, 2888
  HIP_CYLINDER_MIN_LENGTH, HIP_CYLINDER_MAX_LENGTH);

StringPot* thigh_pot = new StringPot(
  THIGH_SENSOR_PIN, adc, ADC_1,
  2176, 58688,  // 136, 3668
  THIGH_CYLINDER_MIN_LENGTH, THIGH_CYLINDER_MAX_LENGTH);

StringPot* knee_pot = new StringPot(
  KNEE_SENSOR_PIN, adc, ADC_0,
  9472, 59776,  //592, 3736
  KNEE_CYLINDER_MIN_LENGTH, KNEE_CYLINDER_MAX_LENGTH);

//StringPot* compliant_pot = new StringPot(
//  COMPLIANT_SENSOR_PIN, adc, ADC_0,
//  0, 65535,
//  ,);

Angle3D joint_angles;
Point3D foot_position;

JointAngleTransform* hip_joint = new JointAngleTransform(
  HIP_A, HIP_B, HIP_ZERO_ANGLE);

JointAngleTransform* thigh_joint = new JointAngleTransform(
  THIGH_A, THIGH_B, THIGH_ZERO_ANGLE);

JointAngleTransform* knee_joint = new JointAngleTransform(
  KNEE_A, KNEE_B, KNEE_ZERO_ANGLE);

Kinematics* kinematics = new Kinematics(false);


void setup() {
  // setup adcs with 8x averaging 12 bit resolution
  adc->setAveraging(8, ADC_0);
  adc->setResolution(16, ADC_0);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_0);
  
  adc->setAveraging(8, ADC_1);
  adc->setResolution(16, ADC_1);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_1);
}

void loop() {
  // read pots
  hip_pot->read_length();
  thigh_pot->read_length();
  knee_pot->read_length();

  joint_angles.hip = hip_joint->length_to_angle(hip_pot->get_length());
  joint_angles.thigh = thigh_joint->length_to_angle(thigh_pot->get_length());
  joint_angles.knee = knee_joint->length_to_angle(knee_pot->get_length());

  kinematics->angles_to_xyz(joint_angles, &foot_position);

  #ifdef PLOT_VALUES
  // report adc value
  Serial.print(hip_pot->get_adc_value()); Serial.print(", ");
  Serial.print(thigh_pot->get_adc_value()); Serial.print(", ");
  Serial.print(knee_pot->get_adc_value()); Serial.print(", ");
  #endif
  #ifdef PLOT_POTS
  // report cylinder lengths
  Serial.print(hip_pot->get_length()); Serial.print(", ");
  Serial.print(thigh_pot->get_length()); Serial.print(", ");
  Serial.print(knee_pot->get_length()); Serial.print(", ");
  #endif
  #ifdef PLOT_ANGLES
  // report joint angles
  Serial.print(joint_angles.hip); Serial.print(", ");
  Serial.print(joint_angles.thigh); Serial.print(", ");
  Serial.print(joint_angles.knee); Serial.print(", ");
  #endif
  #ifdef PLOT_XYZ
  // report xyz
  Serial.print(foot_position.x); Serial.print(", ");
  Serial.print(foot_position.y); Serial.print(", ");
  Serial.print(foot_position.z); Serial.print(", ");
  #endif
  Serial.println();
  delay(100);
}
