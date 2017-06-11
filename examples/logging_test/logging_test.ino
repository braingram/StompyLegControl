#include "logger.h"

//Logger logger = Logger();

#define N_LEVELS 6
byte levels[] = {0, 10, 20, 30, 40, 50};

void write_log(String msg) {
  Serial.println(msg);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  logger.set_writer(write_log);
  logger.set_level(LOG_LEVEL_DEBUG);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=0; i<N_LEVELS; i++) {
    logger.set_level(levels[i]);
    Serial.print("Set level: "); Serial.println(levels[i]);
    for (int j=0; j<N_LEVELS; j++) {
      logger.log(String(levels[j]), levels[j]);
    }
    delay(1000);
  }
  Serial.println("===========");
  delay(1000);
}
