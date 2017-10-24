#!/usr/bin/env python

import sys

import numpy
import pylab


fn = sys.argv[1]
vs = []
with open(fn, 'r') as f:
    for l in f:
        ts = l.strip().split(',')
        if len(ts) < 2 or ts[1] != 'pid':
            continue
        t, _, ho, to, ko, hs, ts, ks, he, te, ke = ts
        # skip 0.0 error?
        vs.append((
            float(t),
            float(ho), float(to), float(ko),
            float(hs), float(ts), float(ks),
            float(he), float(te), float(ke)))

vs = numpy.array(vs, dtype=[
    ('t', 'f8'),
    ('ho', 'f8'), ('to', 'f8'), ('ko', 'f8'),
    ('hs', 'f8'), ('ts', 'f8'), ('ks', 'f8'),
    ('he', 'f8'), ('te', 'f8'), ('ke', 'f8')])

si = 311
ax = None
for j in ('h', 't', 'k'):
    if ax is None:
        ax = pylab.subplot(si)
    else:
        pylab.subplot(si, sharex=ax)
    pylab.title(j)
    pylab.plot(vs['t'], vs[j + 'e'], color='r', label='error')
    pylab.plot(vs['t'], vs[j + 's'], color='b', label='setpoint')
    pylab.gca().twinx().plot(vs['t'], vs[j + 'o'], color='g', label='output')
    si += 1
pylab.show()
