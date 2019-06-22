#include <kinematics.h>

#define ZERO_ANGLE 0.0001
#define N_POINTS 50

float hip_angle_min = -0.70616022;
float hip_angle_max = 0.70616022;
float hip_angle_step = 0.;

float thigh_angle_min = 0.;
float thigh_angle_max = 1.5708;
float thigh_angle_step = 0.;

float knee_angle_min = -2.3736;
float knee_angle_max = 0.;
float knee_angle_step = 0.;

int angle_to_print = 0;  // hip
int axis_to_print = 0;  // z

Angle3D a;
Point3D p;

Kinematics* k = new Kinematics(true);  // on right side

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //hip_angle_step = (hip_angle_max - hip_angle_min) / N_POINTS;
  //thigh_angle_step = (thigh_angle_max - thigh_angle_min) / N_POINTS;
  //knee_angle_step = (knee_angle_max - knee_angle_min) / N_POINTS;
  
  a.hip = 0.;
  a.thigh = 0.;
  a.knee = 0.;
}

void loop() {
  // put your main code here, to run repeatedly:
  bool valid = k->angles_to_xyz(a, &p);
  if (!valid) {
    Serial.println("Invalid angles_to_xyz");
  } else {
    Angle3D pa;
    valid = k->xyz_to_angles(p, &pa);
    if (!valid) Serial.println("Invalid xyz_to_angles");
    valid = (
      (abs(a.hip - pa.hip) < ZERO_ANGLE) &
      (abs(a.thigh - pa.thigh) < ZERO_ANGLE) &
      (abs(a.knee - pa.knee) < ZERO_ANGLE));
    if (!valid) {
      Serial.println("Invalid reprojection");
    } else {
      switch (angle_to_print) {
        case (0):
          Serial.print(a.hip); Serial.print(",");
          break;
        case (1):
          Serial.print(a.thigh); Serial.print(",");
          break;
        case (2):
          Serial.print(a.knee); Serial.print(",");
          break;
        default:
          Serial.print(a.hip); Serial.print(",");
          Serial.print(a.thigh); Serial.print(",");
          Serial.print(a.knee); Serial.print(",");
      }
      Serial.print(" ");
      switch (axis_to_print) {
        case (0):
          Serial.print(p.x); Serial.print(",");
          break;
        case (1):
          Serial.print(p.y); Serial.print(",");
          break;
        case (2):
          Serial.print(p.z); Serial.print(",");
          break;
        default:
          Serial.print(p.x); Serial.print(",");
          Serial.print(p.y); Serial.print(",");
          Serial.print(p.z); Serial.print(",");
      }
      Serial.print(" ");
      switch (angle_to_print) {
        case (0):
          Serial.print(pa.hip); Serial.print(",");
          break;
        case (1):
          Serial.print(pa.thigh); Serial.print(",");
          break;
        case (2):
          Serial.print(pa.knee); Serial.print(",");
          break;
        default:
          Serial.print(pa.hip); Serial.print(",");
          Serial.print(pa.thigh); Serial.print(",");
          Serial.print(pa.knee); Serial.print(",");
      }
    }
  }
  Serial.println("");
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'h':
        angle_to_print = 0;
        break;
      case 't':
        angle_to_print = 1;
        break;
      case 'k':
        angle_to_print = 2;
        break;
      case 'x':
        axis_to_print = 0;
        break;
      case 'y':
        axis_to_print = 1;
        break;
      case 'z':
        axis_to_print = 2;
        break;
      case 'a':
        axis_to_print = 3;
        angle_to_print = 3;
        break;
      case 'H':
        if (hip_angle_step == 0.0) {
          hip_angle_step = (hip_angle_max - hip_angle_min) / N_POINTS;
        } else {
          hip_angle_step = 0.;
        };
        break;
      case 'T':
        if (thigh_angle_step == 0.0) {
          thigh_angle_step = (thigh_angle_max - thigh_angle_min) / N_POINTS;
        } else {
          thigh_angle_step = 0.;
        };
        break;
      case 'K':
        if (knee_angle_step == 0.0) {
          knee_angle_step = (knee_angle_max - knee_angle_min) / N_POINTS;
        } else {
          knee_angle_step = 0.;
        };
        break;
    }
  }
  a.hip += hip_angle_step;
  if ((a.hip > hip_angle_max) | (a.hip < hip_angle_min)) {
    hip_angle_step *= -1;
    a.hip += hip_angle_step * 2;
  };
  a.thigh += thigh_angle_step;
  if ((a.thigh > thigh_angle_max) | (a.thigh < thigh_angle_min)) {
    thigh_angle_step *= -1;
    a.thigh += thigh_angle_step * 2;
  };
  a.knee += knee_angle_step;
  if ((a.knee > knee_angle_max) | (a.knee < knee_angle_min)) {
    knee_angle_step *= -1;
    a.knee += knee_angle_step * 2;
  };
  delay(100);
}
