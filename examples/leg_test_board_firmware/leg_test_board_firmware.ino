/*
 * Drive a leg box connected to the test leg board to
 * check the sensor wiring
 * 
 * wait for serial command to start procedure
 * to hopefully prevent driving valves when connected to the
 * real leg
 * 
 * will need to set heartbeat
 * drive pwms, examine sensors
 * 
 * for hip thigh knee, divide by 50000 (and clip)
 * for calf divide by 18000 (and clip)
 * 
 * cases:
 *  hip 1.0: thigh high
 *  hip -1.0: hip high
 *  thigh 1.0: knee high
 *  thigh -1.0: thigh high
 *  knee 1.0: calf high
 *  knee -1.0: knee high
 */

#include "leg.h"


Leg* leg = new Leg();
bool is_running = false;

elapsedMillis test_timer;


void setup() {
  Serial.begin(9600);
}

void print_adcs(int v) {
  //Serial.println("----- adcs -----");
  Serial.print(v); Serial.print(", ");
  Serial.print(leg->hip_analog_sensor->get_adc_value());
  Serial.print(", ");
  Serial.print(leg->thigh_analog_sensor->get_adc_value());
  Serial.print(", ");
  Serial.print(leg->knee_analog_sensor->get_adc_value());
  Serial.print(", ");
  Serial.println(leg->calf_analog_sensor->get_adc_value());
  
}

int run_tests() {
  Serial.println("=== start of test ===");
  elapsedMillis adc_settle_timer;
  // start hearbeat
  leg->estop->set_heartbeat();
  // disable estop
  leg->estop->set_estop(0);

  for (int t=0; t<6; t++) {
    // turn on hip pwm
    //print_adcs();
    switch (t) {
      case 0:
        leg->hip_valve->set_ratio(1.0);
        break;
      case 1:
        leg->hip_valve->set_ratio(-1.0);
        break;
      case 2:
        leg->thigh_valve->set_ratio(1.0);
        break;
      case 3:
        leg->thigh_valve->set_ratio(-1.0);
        break;
      case 4:
        leg->knee_valve->set_ratio(1.0);
        break;
      case 5:
        leg->knee_valve->set_ratio(-1.0);
        break;
    }
    //Serial.println("Setting hip to 1.0");
    for (int i=0; i<20; i++) {
      // report sensors
      adc_settle_timer = 0;
      while (adc_settle_timer < 50) {
        leg->update();
      }
      print_adcs(t * 1000 + 10000);
      delay(50);
      leg->estop->set_heartbeat();
    };
  
    //Serial.println("Setting hip to 0.0");
    leg->hip_valve->set_ratio(0.0);
    leg->thigh_valve->set_ratio(0.0);
    leg->knee_valve->set_ratio(0.0);
    for (int i=0; i<20; i++) {
      // report sensors
      adc_settle_timer = 0;
      while (adc_settle_timer < 50) {
        leg->update();
      }
      print_adcs(t * 1000);
      delay(50);
      leg->estop->set_heartbeat();
    };
  };
  
  leg->estop->set_estop(2);
}

void print_test_result(int result) {
  //
}

void loop() {
  if (is_running) {
    int r = run_tests();
    print_test_result(r);
  }
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'r': {
      // read leg number
      Serial.print("Reading leg number: ");
      Serial.println((int)(leg->leg_number));
      }; break;
    case 'w': {
      // write leg number
      int n = Serial.parseInt();
      if ((n > 0) & (n <= 255)) {
        Serial.print("Setting leg number: ");
        leg->set_leg_number((LEG_NUMBER)(n));
        Serial.print("set to "); Serial.println(n);
      } else {
        Serial.print("Invalid leg number: ");
        Serial.println(n);
      };
      }; break;
    case 't': {
      is_running = !is_running;
      Serial.print("Running: "); Serial.println(is_running);
      }; break;
  }
}
