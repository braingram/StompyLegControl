#include <legControl.h>

legControl leg1;

        float angles[] = {0, 80, 100};
        float xyz[3];
        int sGoals[3];

void setup() {
        Serial.begin(9600); 

}

void loop() {
        float a[] = {1, 79, 99};
\
        for (int i; i < 3; i++) {
                Serial.println("joint: ");
                Serial.print(i);
                Serial.print("\t");
                Serial.print("angle: ");
                Serial.print("\t");
                Serial.print(a[i]);
                Serial.print("\t");
                Serial.print("sensor: ");
                Serial.print("\t");

                sGoals[i] = leg1.angleToSensor(i, a[i]);
                Serial.print(sGoals[i]);
                Serial.println();
        }
        
        printAnglesAndSensors();

        Serial.print("knee 13 deg:");
        Serial.print("\t");
        Serial.print(leg1.angleToSensor(2, 13));
        Serial.print("\t");

        // Serial.print("knee 120 deg:");
        // Serial.print("\t");
        // Serial.println(leg1.angleToSensor(2, 120));

        // Serial.print("thigh 86 deg:");
        // Serial.print("\t");
        // Serial.print(leg1.angleToSensor(1, 86));
        // Serial.print("\t");

        // Serial.print("thigh 0 deg:");
        // Serial.print("\t");
        // Serial.println(leg1.angleToSensor(1, 0));

        // Serial.print("hip -42 deg:");
        // Serial.print("\t");
        // Serial.print(leg1.angleToSensor(0, -42));
        // Serial.print("\t");

        // Serial.print("hip 42 deg:");
        // Serial.print("\t");
        // Serial.println(leg1.angleToSensor(0, 42));

        delay(2000);
}


void printAnglesAndSensors() {

        for (int i; i < 3; i ++) {
                Serial.print(sGoals[i]);
                Serial.print("\t");
        }

        //Serial.printf("angles HTK: %g %g %g\t", angles[0], angles[1], angles[2]);
        //Serial.printf("goal sensors: %f %f %f\n", sGoals[0], sGoals[1], sGoals[2]);

}