#!/usr/bin/python

import velchange as v
import time

SPEED = 0.4
SLEEP_TIME = 2

v.goXYOmega(SPEED, 0, 0) #forward
time.sleep(SLEEP_TIME/2)
v.goXYOmega(0, 0, 0) # stop
time.sleep(SLEEP_TIME)
v.goXYOmega(0, SPEED, 0) #right
time.sleep(SLEEP_TIME/2)
v.goXYOmega(0, 0, 0) # stop
time.sleep(SLEEP_TIME)
v.goXYOmega(-SPEED, 0, 0) #back
time.sleep(SLEEP_TIME/2)
v.goXYOmega(0, 0, 0) # stop
time.sleep(SLEEP_TIME)
v.goXYOmega(0, -SPEED, 0) #left
time.sleep(SLEEP_TIME/2)
v.goXYOmega(0, 0, 0) # stop
time.sleep(SLEEP_TIME)
v.goXYOmega(SPEED, 0, 0) #forward
time.sleep(SLEEP_TIME/2)
v.goXYOmega(0, 0, 0) # stop
time.sleep(SLEEP_TIME)
v.goXYOmega(0, 0, 10) #victory spin
time.sleep(SLEEP_TIME*2)
v.goXYOmega(0, 0, 0) # stop