/*
	cylinder.h - Library for controlling a cylinder.
	Created by Brett Graham, May 7, 2017.
	Released into the public domain -- so help you God.
*/

#ifndef CYLINDER_H
#define CYLINDER_H

#include "Arduino.h"
#include "valve.h"
#include "string_pot.h"


class Cylinder {
  public:
    Cylinder(Valve* valve, StringPot* string_pot);

    // TODO add code for soft limits
    // TODO add code for closed loop control
    void update();
  private:
    Valve* _valve;
    StringPot* _pot;
};

#endif
