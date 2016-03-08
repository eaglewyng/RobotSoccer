import math
import time
from Point import Point

PIXELS_PER_METER = 191.0
HEIGHT_CAMERA = 3.0099 #2.921
HEIGHT_ROBOT = .116332

# conversion functions
timeToInt = lambda x: (x.secs-1420000000)*100 + x.nsecs/10000000
pixelToMeter = lambda x: x/PIXELS_PER_METER
meterToPixel = lambda x: int(x*PIXELS_PER_METER)
degreeToRadian = lambda x: x/180.0 * math.pi
radianToDegree = lambda x: int(x * 180.0 / math.pi)

def getTime():
  return int((time.time()-1420000000)*100.0)

MAX_SPEED = 2
MIN_SPEED = .1
MAX_OMEGA = 20
MIN_OMEGA = 2
SCALE_VEL = 1.0
SCALE_OMEGA = 0.3
RUSH_SPEED = .3
CIRCLE_SPEED = .3

SPEED_ROBOT = .6 # part of deprecated function.
SPEED_ROTATION = 1.0 # part of deprecated function.
DIS_BEHIND_BALL = 0.15
HOME_GOAL = Point(pixelToMeter(350),0)
AWAY_GOAL = Point(pixelToMeter(-350),0)
CENTER = Point();

HEIGHT_FIELD = 500
HEIGHT_FIELD_METER = pixelToMeter(HEIGHT_FIELD)
WIDTH_FIELD = 700
GUI_MARGIN = 10

HEIGHT_GOAL = 100
WIDTH_GOAL = 30

WIDTH_BALL = meterToPixel(.04178)
RADIUS_ROBOT = meterToPixel(.10124)

GUI_CENTER_X = GUI_MARGIN + WIDTH_FIELD/2
GUI_CENTER_Y = GUI_MARGIN + HEIGHT_FIELD/2

ROBOT_DIAMETER = .2032
MARGIN = DIS_BEHIND_BALL + ROBOT_DIAMETER/2
CENTER_THRESHOLD = 0.3