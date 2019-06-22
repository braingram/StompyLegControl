#!/usr/bin/env python
"""
Take in:
- joint name
- actual min and max readings
- actual min and max distances

compute min and max sensor values for
actual min and max values
y = mx + b where x = length in inches, y = sensor reading

m is sensor units per inch (smax - smin / dmax - dmin)
b = smax - m * dmax

cmin = amin * m + b
cmax = amax * m + b
"""

# fr 170904
joint_name = 'hip'
smin = 5909
smax = 46363
dmin = 15.957
dmax = 23.0195
amin = 16.0
amax = 24.0

# fr 170904
joint_name = 'thigh'
smin = 13557
smax = 40026
dmin = 26.7175
dmax = 33.06125
amin = 24.0
amax = 38.0

# fr 170904
joint_name = 'knee'
smin = 6730
smax = 48474
dmin = 20.7835
dmax = 30.6585
amin = 20.0
amax = 32.0

m = (smax - smin) / (dmax - dmin)
b = smax - m * dmax
cmin = amin * m + b
cmax = amax * m + b
print(joint_name)
print(cmin)
print(cmax)
print((cmin - b) / m)
print((cmax - b) / m)
