#include "ADC.h"
#include "valve.h"
#include "string_pot.h"
#include "cylinder.h"

ADC* adc = new ADC();
Valve* valve = new Valve(3, 4, 0);
StringPot* pot = new StringPot(adc, A0, 10, 20, 5.0, 10.0);
Cylinder* cylinder = new Cylinder(valve, pot);

void setup() {
  adc->setAveraging(8);
  adc->setResolution(16);
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED);
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED);
}

void loop() {
}
