#include <legControl.h>

legControl leg1;

void setup() {
        Serial.begin(9600); 
        // put your setup code here, to run once:

}

void loop() {


        float angles[3];
        float xyz[3];
        angles[0] = 0;
        angles[1] = 80;
        angles[2] = 100;
        float anglesRad[3];
        leg1.anglesToRad(angles, anglesRad);
        leg1.anglesToXYZ(angles, xyz);
        Serial.print("angles in:");
        for (int i = 0; i < 3; i ++) {
                Serial.print("\t");
                Serial.print(angles[i]);  
        }
        Serial.println();
        Serial.print("calculated xyz:");
        for (int i = 0; i < 3; i ++) {
                Serial.print("\t");
                Serial.print(xyz[i]);  
        } 
        Serial.println();

        // for (int joint = 0; joint < 3; joint ++) {
        //         for (int sensorReading = 100; sensorReading <= 800; sensorReading = sensorReading + 10) {
        //                 //use libragy to calculate angle from sensor reading then do a round trip with that reading back to angle.
        //                 float angle = leg1.sensorToAngle(joint, sensorReading);
        //                 int sensor = leg1.angleToSensor(joint, angle);
        //                 //Compare the cacluation loop to make sure we get the correct sensor reading out
        //                 int loLimit = sensorReading - 1;
        //                 int hiLimit = sensorReading + 1;
        //                 if (sensor >= loLimit && sensor <= hiLimit) {
        //                         Serial.print("PASS");
        //                 }
        //                 else {
        //                         Serial.print("FAIL");
        //                 }
        //                 Serial.print("\t");
        //                 Serial.print("joint:");
        //                 Serial.print("\t");
        //                 Serial.print(joint);
        //                 Serial.print("\t");
        //                 Serial.print("sensor in:");
        //                 Serial.print("\t");
        //                 Serial.print(sensorReading);
        //                 Serial.print("\t");
        //                 Serial.print("calculated angle:");
        //                 Serial.print("\t");
        //                 Serial.print(angle);
        //                 Serial.print("\t");
        //                 Serial.print("calculated sensor:");
        //                 Serial.print("\t");
        //                 Serial.print(sensor); 
        //                 Serial.println();
        //                 //now compare the cacluation loop to make sure we get the correct sensor reading out

        //                 delay(1000);
        //         }
        // }

        delay(1000);
}