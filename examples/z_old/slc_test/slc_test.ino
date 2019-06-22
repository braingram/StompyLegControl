#include "ADC.h"
#include "valve.h"
#include "sensors.h"
#include "cylinder.h"

ADC* adc = new ADC();

StringPot* hipPot = new StringPot(A0, adc, ADC_0, 10, 20, 5.0, 10.0);
StringPot* thighPot = new StringPot(A1, adc, ADC_0, 10, 20, 5.0, 10.0);
StringPot* kneePot = new StringPot(A2, adc, ADC_0, 10, 20, 5.0, 10.0);;
// TODO compliant link

Valve* hipValve = new Valve(3, 4, 0);
Valve* thighValve = new Valve(3, 4, 0);
Valve* kneeValve = new Valve(3, 4, 0);

Cylinder* hipCylinder = new Cylinder(hipValve, hipPot);
Cylinder* thighCylinder = new Cylinder(thighValve, thighPot);
Cylinder* kneeCylinder = new Cylinder(kneeValve, kneePot);

/*
Joint* hipJoint = new Joint(hipCylinder);
Joint* thighJoint = new Joint(hipCylinder);
Joint* kneeJoint = new Joint(hipCylinder);
*/

void setup() {
  adc->setAveraging(8, ADC_0);
  adc->setResolution(16, ADC_0);
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_0);
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_0);

  adc->setAveraging(8, ADC_1);
  adc->setResolution(16, ADC_1);
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1);
}

void loop() {
}
