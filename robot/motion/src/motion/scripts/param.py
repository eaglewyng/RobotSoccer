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

MAX_SPEED = 1.0
MIN_SPEED = .1
MAX_OMEGA = 8
MIN_OMEGA = 2
SCALE_VEL = 1.0
SCALE_OMEGA = 0.3
RUSH_SPEED = .3
CIRCLE_SPEED = .3

SPEED_ROBOT = .6 # part of deprecated function.
SPEED_ROTATION = 1.0 # part of deprecated function.
HOME_GOAL = Point(pixelToMeter(320),0)
AWAY_GOAL = Point(pixelToMeter(-320),0)
CENTER = Point()
START_LOC = Point(pixelToMeter(100), 0)

HEIGHT_FIELD = 430
HEIGHT_FIELD_METER = pixelToMeter(HEIGHT_FIELD)
WIDTH_FIELD = 640
GUI_MARGIN = 10

HEIGHT_GOAL = 100
WIDTH_GOAL = 30

WIDTH_BALL = meterToPixel(.04178)
RADIUS_ROBOT = meterToPixel(.10124)

GUI_CENTER_X = GUI_MARGIN + WIDTH_FIELD/2
GUI_CENTER_Y = GUI_MARGIN + HEIGHT_FIELD/2

ROBOT_DIAMETER = 0.18
DIS_BEHIND_BALL = ROBOT_DIAMETER/2.0 + .02
MARGIN = DIS_BEHIND_BALL + ROBOT_DIAMETER/2
MOVEMENT_THRESHOLD = ROBOT_DIAMETER/4
