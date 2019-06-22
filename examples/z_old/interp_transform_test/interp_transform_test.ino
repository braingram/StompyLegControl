#include "transforms.h"

float pts[] = {
  0.0000,
 0.5878,
 0.9511,
 0.9511,
 0.5878,
 0.0000,
 -0.5878,
 -0.9511,
 -0.9511,
 -0.5878,
 -0.0000
  };
int n_pts = sizeof(pts) / sizeof(float);

float smin = -1.0;
float smax = 1.0;

int interp_n_pts = 101;

InterpolatedTransform* t = new InterpolatedTransform(
  smin, smax, pts, n_pts
  );

void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i=0; i<interp_n_pts; i++) {
    float f = i / (interp_n_pts - 1.0) * (smax - smin) + smin;
    float fi = t->src_to_dst(f);
    Serial.print(f);
    Serial.print(" ");
    Serial.print(fi);
    Serial.println();
    delay(10);
  }
}
