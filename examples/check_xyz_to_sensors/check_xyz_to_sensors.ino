#include <legControl.h>

legControl leg1;

        float startingAngles[3] = {0, 80, 100};
        float goalAngles[3] = {30, 60, 20};
        float startingXYZ[3];
        float goalXYZ[3];

        int sensorGoals[3];

void setup() {
        Serial.begin(9600); 
        // put your setup code here, to run once:

}

void loop() {

        leg1.anglesToXYZ(startingAngles, startingXYZ);
        leg1.anglesToXYZ(goalAngles, goalXYZ);
        
        //int startingSensors[3];
        //int goalSensors[3];
        float sensorVelocities[3];
        float move_time = 10; //in seconds

        leg1.goal_XYZ_toSensorVelocities(startingXYZ, goalXYZ, sensorVelocities, move_time);

        

        Serial.print("sensor velocities: ");
        for (int i = 0; i < 3; i ++) {
                Serial.print(sensorVelocities[i]);
                Serial.print("\t");
        }
        Serial.println();
        
       

        delay(2000);
}