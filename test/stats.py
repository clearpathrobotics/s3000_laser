#!/usr/bin/env python

from re import findall

with open('laser-14.05s.log') as f:
    t = f.read()

print len(t), "bytes"
print len(findall("\0\0\0\0\0\0", t))

