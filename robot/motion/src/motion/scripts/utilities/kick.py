#!/usr/bin/env python
import os

def initkick():
  os.system("echo 30 > /sys/class/gpio/export")
  os.system("echo out > /sys/class/gpio/gpio30/direction")
  os.system("echo 0 > /sys/class/gpio/gpio30/value")

def kick():
  os.system("echo 1 > /sys/class/gpio/gpio30/value; sleep .1; echo 0 > /sys/class/gpio/gpio30/value")

if __name__ == '__main__':
  initkick()
  kick()
