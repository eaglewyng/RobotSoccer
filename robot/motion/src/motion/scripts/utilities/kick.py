#!/usr/bin/env python
import os
def initkick():
  os.system("echo 29 > /sys/class/gpio/export")
  os.system("echo out > /sys/class/gpio/gpio29/direction")
  
def kick():
  os.system("echo 1 > /sys/class/gpio/gpio29/value; sleep .1; echo 0 > /sys/class/gpio/gpio29/value")

if __name__ == '__main__':
  kick()
