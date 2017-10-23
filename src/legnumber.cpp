#include "legnumber.h"

void write_leg_number_to_eeprom(LEG_NUMBER n) {
  EEPROM.write(LEGNUMBER_ADDR, (byte)(n));
};

LEG_NUMBER read_leg_number_from_eeprom() {
  byte v = EEPROM.read(LEGNUMBER_ADDR);
  if (v > LEGNUMBER_MAX) v = (byte)LEG_NUMBER::UNDEFINED;
  return (LEG_NUMBER)(v);
};
