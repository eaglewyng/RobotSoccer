#!/usr/bin/env python

import math
import rospy
import sys
import time
from robot_soccer.srv import *
from robot_soccer.msg import *
from MotionSkills import *
from motor_control import roboclaw
from gamepieces.HomeRobot import HomeRobot
from gamepieces.HomeRobot import RobotCommand
from gamepieces.Ball import *
from param import *
from utilities.kick import *
from kalman_filter.Locations import *
import sched
import cPickle as pickle

from enum import Enum
class GameState(Enum):
  stop = 1
  play = 2
  center = 3
  startPosition = 4
  test = 5
  goToPoint = 6

class PlayState(Enum):
  rushGoal = 1
  getBehindBall = 2
  rotateToAngleBehindBall = 3
  check = 4
  returnToPlay = 5

class Strategy(Enum):
  DEFENSIVE = 1
  OFFENSIVE = 2
  NONE = 3

class Vektory:
  def __init__(self):
    self.locations = None
    self.ball = Ball()
    self.robotLocation = None
    self.clickLocation = Point()
    self.distanceToBall = 0
    self.playState = PlayState.check
    self.stopRushingGoalTime = 0
    self.gameState = GameState.stop

    #figure out which robot I am
    self.robotAssignment = rospy.get_param('robot', 1)
    if self.robotAssignment != 1 and self.robotAssignment != 2:
      print("Invalid robot assignemnt! Assigning 1")
      self.robotAssignment = 1

    self.lastBall = Ball()
    self.lastHome1 = RobotLocation()
    self.lastTimeStamp = -1
    self.currBallXVel = 0
    self.currBallYVel = 0

    initkick()

    self.integrator = {'x': 0, 'y': 0, 'theta': 0}
    self.differentiator = {'x': 0, 'y': 0, 'theta': 0}
    self.error_d1 = {'x': 0, 'y': 0, 'theta': 0}

    self.strategy = Strategy.DEFENSIVE

  def sendCommand(self, vel_x, vel_y, omega, theta = 0):
    command = RobotCommand(-1, vel_x, vel_y, omega, theta)
    command.execute()
    sendCommand = rospy.ServiceProxy('commandSent', commandsent)
    # try:
    #   resp1 = sendCommand(*(command.getCommandToSend()))
    # except rospy.ServiceException as exc:
    #   print("Service did not process request: " + str(exc))

  def rotateAroundBallToAngle(self,targetAngle):
    #lookAtPoint = self.ball.point
    delta = MotionSkills.deltaBetweenAngles(self.robotLocation.theta, targetAngle)
    vektor_x = 0
    vektor_y = 0
    if abs(delta) < .1:
      self.playState = PlayPlayState.check
      return
    if delta > 0:
      vektor_y = -CIRCLE_SPEED
    else:
      vektor_y = CIRCLE_SPEED

    #angle = math.atan2(lookAtPoint.y-self.robotLocation.y, lookAtPoint.x-self.robotLocation.x)

    #delta_angle = angle-self.robotLocation.theta

    #bestDelta = math.atan2(math.sin(delta_angle), math.cos(delta_angle)) * SCALE_OMEGA
    #print bestDelta

    #if bestDelta < MIN_OMEGA and bestDelta > -MIN_OMEGA:
    #  bestDelta = 0
    good = CIRCLE_SPEED/self.distanceToBall
    good_y = 0
    if delta > 0:
      good_y = -CIRCLE_SPEED
    else:
      good_y = CIRCLE_SPEED
    self.sendCommand(0, good_y, good, 0)
    #self.sendCommand(vektor_x, vektor_y, bestDelta, 0)

  def go_to_point(self,x, y, lookAtPoint=None):
    if lookAtPoint == None:
      lookAtPoint = self.ball.point
    desired_theta = math.atan2(lookAtPoint.y-self.robotLocation.y, lookAtPoint.x-self.robotLocation.x)
    # print("look point = {},{}, my point = {},{}, ANGLE: {}".format(lookAtPoint.x, lookAtPoint.y, self.robotLocation.x, self.robotLocation.y, desired_theta*180/math.pi))

    vektor_x = self.pidloop(x, self.robotLocation.x, 'x')
    vektor_y = self.pidloop(y, self.robotLocation.y, 'y')
    vektor_w = self.pidloop(desired_theta, self.robotLocation.theta, 'theta')
    print("world vel (x, y, w) = ({}, {}, {})").format(vektor_x, vektor_y, vektor_w)
    self.sendCommand(vektor_x, vektor_y, vektor_w, self.robotLocation.theta)
    return

  def updateLocations(self, data):
    try:
        self.locations = Locations()
        self.locations.setLocationsFromMeasurement(data)
        self.robotLocation = self.locations.home1
        self.ball.point.x = self.locations.ball.x
        self.ball.point.y = self.locations.ball.y
        self.distanceToBall = distance(self.robotLocation, self.locations.ball)
        #update predictions
        self.updatePredictions()
    except rospy.ServiceException, e:
        print "service call failed: %s"%e

  def defensiveStrats(self):
    predBall = self.ballPrediction(1.5)
    lookAtPoint = self.ball.point
    DEFENSIVE_X_COORD = HOME_GOAL.x - 0.2
    DEFENSIVE_Y_COORD = predBall.y

    # don't leave the goalie box
    DEFENSIVE_Y_COORD = min(DEFENSIVE_Y_COORD, HEIGHT_FIELD_METER/4)
    DEFENSIVE_Y_COORD = max(DEFENSIVE_Y_COORD, -HEIGHT_FIELD_METER/4)

    # only guard if the ball isn't past the robot
    if self.ball.point.x < self.robotLocation.x:
      self.go_to_point(DEFENSIVE_X_COORD, DEFENSIVE_Y_COORD, lookAtPoint)
    else:
      self.go_to_point(DEFENSIVE_X_COORD, HOME_GOAL.y, lookAtPoint)

  def play(self):
    #print (self.robotLocation.x, self.robotLocation.y)
    #self.setSpeed()
    #check if robot is ready to rush goal
    if self.playState == PlayState.check:
      self.playState = PlayState.getBehindBall
      if self.robotLocation.x > pixelToMeter(345):
        self.playState = PlayState.returnToPlay
      elif (MotionSkills.isPointInFrontOfRobot(self.robotLocation,self.ball.point, 0.5, 0.04 + abs(MAX_SPEED/4))): #This offset compensates for the momentum
        print("REALLY BEHIND BALL")
        self.playState = PlayState.rushGoal# rush goal
        self.stopRushingGoalTime = getTime() + int(2 * DIS_BEHIND_BALL/MAX_SPEED*100)

    if self.playState == PlayState.rushGoal:
      print("RUSHING")
      #self.speed = RUSH_SPEED
      self.go_to_point(AWAY_GOAL.x, AWAY_GOAL.y, AWAY_GOAL)
      if getTime() >= self.stopRushingGoalTime:
        kick()
        print("KICKED")
        self.playState = PlayState.check

    if self.playState == PlayState.returnToPlay:
      self.go_to_point(CENTER.x, CENTER.y, AWAY_GOAL)
      if abs(self.robotLocation.x) < .2 and abs(self.robotLocation.y) < .2:
        self.playState = PlayState.check

    #check if ball is behind robot
    if self.playState == PlayState.getBehindBall:
      predBallLoc = self.ballPrediction(time.time() - self.lastTimeStamp)
      desiredPoint = MotionSkills.getPointBehindBallXY(predBallLoc.x, predBallLoc.y, home_goal=AWAY_GOAL)
      distFromPoint = distance(self.robotLocation, desiredPoint)
      if distFromPoint < 0.1:
        self.playState = PlayState.rushGoal
        self.stopRushingGoalTime = getTime() + 50
        kick()
      elif self.ball.point.x > AWAY_GOAL:
        #try go to point
        self.go_to_point(desiredPoint.x, desiredPoint.y)
        #self.go_direction(desiredPoint)

  def jarjar_oneRobotStrategy(self):
    #if (robot is on the bottom quarter of our field)
    # goFullDefensive()
    #if (robot is anywhere else)
    # goFullOffensive()
    if self.ball.point.x < WIDTH_FIELD/4:
      self.strategy = Strategy.OFFENSIVE
      self.play()
    else:
      self.strategy = Strategy.DEFENSIVE
      self.defensiveStrats()

  def scoreGoal(self):
    #1. get behind ball


    #2. get ball into goal

  def executionLoop(self, scheduler):
    scheduler.enter(0.1, 1, self.executionLoop,(scheduler,))
    if self.gameState == GameState.play:
      self.jarjar_oneRobotStrategy()
    elif self.gameState == GameState.test:
      self.defensiveStrats()
      self.strategy = Strategy.DEFENSIVE
    elif self.gameState == GameState.stop:
      self.strategy = Strategy.NONE
      self.sendCommand(0, 0, 0)
    elif self.gameState == GameState.center:
      self.strategy = Strategy.NONE
      if distance(self.robotLocation, Point(CENTER.x, CENTER.y)) > MOVEMENT_THRESHOLD:
        self.go_to_point(CENTER.x, CENTER.y, None)
      else:
        self.sendCommand(0, 0, 0)
    elif self.gameState == GameState.startPosition:
      self.strategy = Strategy.NONE
      if distance(self.robotLocation, START_LOC) > MOVEMENT_THRESHOLD or abs(self.robotLocation.theta) > 0.1:
        self.go_to_point(START_LOC.x, START_LOC.y, AWAY_GOAL)
      else:
        self.sendCommand(0, 0, 0)
    elif self.gameState == GameState.goToPoint:
      self.strategy = Strategy.NONE
      if distance(self.robotLocation, self.clickLocation) > MOVEMENT_THRESHOLD:
        self.go_to_point(self.clickLocation.x, self.clickLocation.y)
      else:
        self.sendCommand(0, 0, 0)

  def executeCommCenterCommand(self,req):
    self.resetPIDValues()
    if req.comm == 1:
      self.gameState = GameState.stop
      self.playState = PlayState.check
    elif req.comm == 2:
      self.gameState = GameState.play
    elif req.comm == 3:
      self.gameState = GameState.center
      self.playState = PlayState.check
    elif req.comm == 4:
      self.gameState = GameState.startPosition
      self.playState = PlayState.check
    elif req.comm == 5:
      self.gameState = GameState.test
    elif req.comm == 6:
      self.gameState = GameState.goToPoint
      self.clickLocation = Point(pixelToMeter(req.x), pixelToMeter(req.y))
    return commcenterResponse()

  def go(self):
    # roboclaw.calibrateRoboclaws()
    print "Searching for Vektor Locations Service..."
    rospy.wait_for_service('locations')
    print "Searching for Vektor Locations Update Service..."
    rospy.wait_for_service('commandSent')
    print "Entering Gameplay"
    rospy.init_node('vektoryNode')
    rospy.Service('commcenter', commcenter, self.executeCommCenterCommand)
    s = sched.scheduler(time.time, time.sleep)
    rospy.Subscriber("locTopic", locations, self.updateLocations)
    print "Subscribed to locTopic!"
    s.enter(0,1,self.executionLoop,(s,))
    s.run()

  def updatePredictions(self):
    # get new timestamp
    newTimeStamp = time.time()
    sample_period = newTimeStamp - self.lastTimeStamp
    self.lastTimeStamp = newTimeStamp

    # update ball prediction
    ball_vector_x = (self.ball.point.x - self.lastBall.point.x) #x distance traveled
    ball_vector_y = (self.ball.point.y - self.lastBall.point.y) #y distance traveled
    ball_mag = math.sqrt(ball_vector_x**2 + ball_vector_y**2)
    ball_angle = math.atan2(ball_vector_y, ball_vector_x)
    ball_velocity = ball_mag/sample_period

    self.currBallXVel = ball_vector_x / sample_period
    self.currBallYVel = ball_vector_y / sample_period

    self.lastBall.point.x = self.ball.point.x
    self.lastBall.point.y = self.ball.point.y

    # update robot prediction
    home1_vector_x = (self.robotLocation.x - self.lastHome1.x) #x distance traveled
    home1_vector_y = (self.robotLocation.y - self.lastHome1.y) #y distance traveled
    home1_mag = math.sqrt(home1_vector_x**2 + home1_vector_y**2)
    home1_angle = math.atan2(home1_vector_y, home1_vector_x)
    home1_velocity = home1_mag/sample_period

    # print(home1_velocity)

    self.currHome1XVel = home1_vector_x / sample_period
    self.currHome1YVel = home1_vector_y / sample_period

    self.lastHome1.x = self.robotLocation.x
    self.lastHome1.y = self.robotLocation.y

  def ballPrediction(self, time_sec):
    #time_sec is saying where will the ball be in 'time_sec' amount of seconds
    ball_new_position_x = self.ball.point.x + self.currBallXVel*time_sec
    ball_new_position_y = self.ball.point.y + self.currBallYVel*time_sec
    return Point(ball_new_position_x, ball_new_position_y)

  def resetPIDValues(self):
    for key in self.integrator:
      self.integrator[key] = 0
    for key in self.differentiator:
      self.integrator[key] = 0

  def pidloop(self, dest_loc, cur_loc, var):
    def getConstants(var):
      if var == 'x' or var == 'y':
        return (0.01, 0.05, 1.5, 0.0, 0.7, MAX_SPEED)
      elif var == 'theta':
        return (0.01, 0.05, 1.7, 0.0, .7, MAX_OMEGA)
    def sat(x, limit):
      out = max(x, -limit)
      out = min(out, limit)
      return out

    Ts, tau, kp, kd, ki, limit = getConstants(var)

    #compute current error
    if var == 'x' or var == 'y':
      error = dest_loc - cur_loc
    elif var == 'theta':
      # do weird normalization of angles
      dest_loc = dest_loc % (2*math.pi)
      error = dest_loc - cur_loc
      if abs(error) > math.pi:
        if error > 0:
          error = error - 2*math.pi
        else:
          error = error + 2*math.pi
      # negate error because positive rotation of our robot is backwards
      error = -error

    # print("PID cur = {}, dest = {}, error = {}".format(cur_loc, dest_loc, error))
    #update integrator
    self.integrator[var] = self.integrator[var] + (Ts/2) * (error + self.error_d1[var])

    #update differentiator
    self.differentiator[var] = ((2*tau - Ts)/(2*tau + Ts))*self.differentiator[var] + (2/(2*tau + Ts))*(error - self.error_d1[var])

    #u is the output velocity in a specified direction (x or y)
    velocity = sat(kp*error + ki*self.integrator[var] + kd*self.differentiator[var], limit)

    if ki != 0:
      velocity_unsat = kp*error + ki*self.integrator[var] + kd*self.differentiator[var]
      self.integrator[var] = self.integrator[var] + Ts/ki *(velocity - velocity_unsat)

    #u is the output velocity in a specified direction (x or y)
    # print("PID", var, error, velocity)

    MIN_SPEED_XY = 0.7
    MIN_SPEED_THETA = 4
    THRESHOLD = 0.2

    if math.sqrt(self.currHome1XVel**2 + self.currHome1YVel**2) < 0.2:
      if var == 'x' or var == 'y':
        # print("increase {}".format(var))
        if abs(velocity) > THRESHOLD:
          velocity = MIN_SPEED_XY if velocity > 0 else -MIN_SPEED_XY
      # elif var == 'theta':
      #   velocity = velocity + (MIN_SPEED_THETA if velocity > 0 else -MIN_SPEED_THETA)
    return velocity

def distance(cur_loc, dest_loc):
  return math.sqrt((cur_loc.x - dest_loc.x)**2 + (cur_loc.y - dest_loc.y)**2)

if __name__ == '__main__':
  winner = Vektory()
  winner.go()
