#!/usr/bin/python

import velchange as v
import time

v.goXYOmega(0, 0, 10)
time.sleep(3)
v.goXYOmega(0, 0, 0)