#!/usr/bin/env python

cylinders = {
    'hip': {
        'min': 16.0,
        'stroke': 8.0,
    },
    'thigh': {
        'min': 24.0,
        'stroke': 14.0,
    },
    'knee': {
        'min': 20.0,
        'stroke': 12.0,
    },
}

joint = 'hip'
min_value = 7900
min_length = 16.25
max_value = 42400
max_length = 22.5
#joint = 'thigh'
#min_value = 10600
#min_length = 25.78
#max_value = 42700
#max_length = 33.5625
#joint = 'knee'
#min_value = 12000
#min_length = 20.3125
#max_value = 56400
#max_length = 31

adc_res = 16

cylinder = cylinders[joint]
cylinder['max'] = cylinder['min'] + cylinder['stroke']

# convert from adc_res to 16 bit
if adc_res != 16:
    conv = 2. ** 16. / 2. ** adc_res
    min_value *= conv
    max_value *= conv

# generate linear model
# value = y, length = x
slope = (max_value - min_value) / float(max_length - min_length)
intercept = max_value - slope * max_length

print("joint: %s" % joint)
print("slope: %s" % slope)
print("intercept: %s" % intercept)

# calculate lengths for min and max
cmin_value = cylinder['min'] * slope + intercept
cmax_value = cylinder['max'] * slope + intercept
print("Min: %s" % cmin_value)
print("Max: %s" % cmax_value)
