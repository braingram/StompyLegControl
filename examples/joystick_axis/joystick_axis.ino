/*
  Analog input, analog output, serial output

 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground

 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include "ADC.h"
#include "sensors.h"
#include "transforms.h"

ADC* adc = new ADC();

JoystickAxis* joy_x = new JoystickAxis(
  A0, adc, ADC_0, 0, 510, 520, 1024, -1.0, 0.0, 1.0);
JoystickAxis* joy_y = new JoystickAxis(
  A1, adc, ADC_0, 0, 500, 510, 1024, -1.0, 0.0, 1.0);

/* the same linear transform with deadband transform
 *  might be useful for driving valves by using the inverse
 *  dst_to_src function of the transform
 */
/*
LinearDeadbandTransform* pwm_transform = new LinearDeadbandTransform(
  -1.0, -0.5, 0.5, 1.0, -1.0, 0.0, 1.0);
*/

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  float x = joy_x->read_axis();
  float y = joy_y->read_axis();
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  /*
  Serial.print(pwm_transform->dst_to_src(x));
  Serial.print(" ");
  Serial.print(pwm_transform->dst_to_src(y));
  */
  Serial.println();
  delay(10);
}
