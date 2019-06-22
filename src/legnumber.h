/*
  legnumber.h - Library legnumber class.
  Created by Brett Graham, July 23, 2017.
  Released into the public domain -- so help you God.
*/

#ifndef LEGNUMBER_H
#define LEGNUMBER_H

#include "Arduino.h"
#include <EEPROM.h>

#define LEGNUMBER_ADDR 2047
#define LEGNUMBER_MAX 7

enum class LEG_NUMBER : uint8_t {
  UNDEFINED,
  FL,
  ML,
  RL,
  RR,
  MR,
  FR,
  FAKE
};

void write_leg_number_to_eeprom(LEG_NUMBER n);

LEG_NUMBER read_leg_number_from_eeprom();

#endif
