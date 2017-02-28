#include <legControl.h>

legControl leg1;

void setup() {
        // put your setup code here, to run once:

}

void loop() {
        for (int joint = 0; joint < 3; joint ++) {
                for (int sensorReading = 100; sensorReading <= 800; sensorReading = sensorReading + 10) {
                        //use libragy to calculate angle from sensor reading then do a round trip with that reading back to angle.
                        float angle = leg1.sensorToAngle(joint, sensorReading);
                        int sensor = leg1.angleToSensor(joint, angle);
                        //Compare the cacluation loop to make sure we get the correct sensor reading out
                        int loLimit = sensorReading - 1;
                        int hiLimit = sensorReading + 1;
                        if (sensor >= loLimit && sensor <= hiLimit) {
                                Serial.print("PASS");
                        }
                        else {
                                Serial.print("FAIL");
                        }
                        Serial.print("\t");
                        Serial.print("joint:");
                        Serial.print("\t");
                        Serial.print(joint);
                        Serial.print("\t");
                        Serial.print("sensor in:");
                        Serial.print("\t");
                        Serial.print(sensorReading);
                        Serial.print("\t");
                        Serial.print("calculated angle:");
                        Serial.print("\t");
                        Serial.print(angle);
                        Serial.print("\t");
                        Serial.print("calculated sensor:");
                        Serial.print("\t");
                        Serial.print(sensor); 
                        Serial.println();
                        //now compare the cacluation loop to make sure we get the correct sensor reading out

                        delay(50);
                }
        }
}