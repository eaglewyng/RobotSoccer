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
from velocity import *
import os

from enum import Enum
class GameState(Enum):
  STOP = 1
  PLAY = 2
  CENTER = 3
  START_POSITION = 4
  TEST = 5
  GOTOPOINT = 6

class PlayState(Enum):
  RUSHGOAL = 1
  GET_BEHIND_BALL = 2
  ROTATE_TO_ANGLE_BEHIND_BALL = 3
  CHECK = 4
  RETURN_TO_PLAY = 5

class Strategy(Enum):
  DEFENSIVE = 1
  OFFENSIVE = 2
  NONE = 3

class Vektory:
  def __init__(self):
    self.locations = None
    self.ballLocation = Point()
    self.lastBallLocation = Point()
    self.home1Location = RobotLocation()
    self.lastHome1Location = RobotLocation()
    self.awayTeam1Location = RobotLocation()
    self.awayTeam2Location = RobotLocation()
    self.clickLocation = Point()
    self.distanceToBall = 0
    self.playState = PlayState.CHECK
    self.gameState = GameState.STOP
    self.oldGameState = GameState.STOP
    self.stopRushingGoalTime = 0

    #Set this to 1 if you want the 2 robot strategy
    self.twoRobotStrategyEnabled = 0

    #figure out which robot I am
    self.robotAssignment = rospy.get_param('robot', 1)
    if self.robotAssignment != 1 and self.robotAssignment != 2:
      print("Invalid robot assignemnt! Assigning 1")
      self.robotAssignment = 1

    self.lastVisionUpdatedTimeStamp = -1
    self.lastKickedTimeStamp = -1
    self.ballVelocity = Velocity()
    self.home1Velocity = Velocity()



    initkick()

    self.integrator = {'x': 0, 'y': 0, 'theta': 0}
    self.differentiator = {'x': 0, 'y': 0, 'theta': 0}
    self.error_d1 = {'x': 0, 'y': 0, 'theta': 0}
    self.desired_old = {'x': 0, 'y': 0, 'theta': 0}
    self.strategy = Strategy.DEFENSIVE

  def sendCommand(self, vel_x, vel_y, omega, theta = 0):
    command = RobotCommand(-1, vel_x, vel_y, omega, theta)
    command.execute()
    sendCommand = rospy.ServiceProxy('commandSent', commandsent)
    # try:
    #   resp1 = sendCommand(*(command.getCommandToSend()))
    # except rospy.ServiceException as exc:
    #   print("Service did not process request: " + str(exc))

  def getMyLocation(self):
    if self.robotAssignment == 1:
      return self.home1Location
    else:
      return self.home2Location

  def getTeammateLocation(self):
    if self.robotAssignment == 2:
      return self.home2Location
    else:
      return self.home1location

  def rotateAroundBallToAngle(self,targetAngle):
    #lookAtPoint = self.ballLocation
    delta = MotionSkills.deltaBetweenAngles(self.home1Location.theta, targetAngle)
    vektor_x = 0
    vektor_y = 0
    if abs(delta) < .1:
      self.playState = PlayState.CHECK
      return
    if delta > 0:
      vektor_y = -CIRCLE_SPEED
    else:
      vektor_y = CIRCLE_SPEED

    #angle = math.atan2(lookAtPoint.y-self.home1Location.y, lookAtPoint.x-self.home1Location.x)

    #delta_angle = angle-self.home1Location.theta

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
      lookAtPoint = self.ballLocation

    desired_theta = math.atan2(lookAtPoint.y-self.getMyLocation().y, lookAtPoint.x-self.getMyLocation().x)

    vektor_x = self.pidloop(x, self.getMyLocation().x, 'x')
    vektor_y = self.pidloop(y, self.getMyLocation().y, 'y')
    vektor_w = self.pidloop(desired_theta, self.getMyLocation().theta, 'theta')
    print("world vel (x, y, w) = ({}, {}, {})").format(vektor_x, vektor_y, vektor_w)
    self.sendCommand(vektor_x, vektor_y, vektor_w, self.getMyLocation().theta)

  def updateLocations(self, data):
    try:
        self.locations = Locations()
        self.locations.setLocationsFromMeasurement(data)
        self.home1Location = self.locations.home1
        self.home2Location = self.locations.home2
        self.awayTeam1Location = self.locations.away1
        self.awayTeam2Location = self.locations.away2
        self.ballLocation.x = self.locations.ball.x
        self.ballLocation.y = self.locations.ball.y
        self.distanceToBall = distance(self.getMyLocation(), self.locations.ball)
        #update predictions
        self.updatePredictions()
    except rospy.ServiceException, e:
        print "service call failed: %s"%e

  def defensiveStrats(self):
    predBall = self.ballPrediction(1.5)
    lookAtPoint = self.ballLocation
    if self.twoRobotStrategyEnabled == 1:
      if self.robotAssignment == 1:
        DEFENSIVE_X_COORD = HOME_GOAL.x - 0.2
      else:
        DEFENSIVE_X_COORD = HOME_GOAL.x - 0.8
    else:
      DEFENSIVE_X_COORD = HOME_GOAL.x - 0.2

    DEFENSIVE_Y_COORD = predBall.y

    # don't leave the goalie box
    DEFENSIVE_Y_COORD = min(DEFENSIVE_Y_COORD, HEIGHT_FIELD_METER/4)
    DEFENSIVE_Y_COORD = max(DEFENSIVE_Y_COORD, -HEIGHT_FIELD_METER/4)

    # only guard if the ball isn't past the robot
    if self.ballLocation.x < self.home1Location.x:
      self.go_to_point(DEFENSIVE_X_COORD, DEFENSIVE_Y_COORD, lookAtPoint)
    else:
      if self.ballLocation.y < 0:
        self.go_to_point(DEFENSIVE_X_COORD, predBall.y + .2, lookAtPoint)
      else:
        self.go_to_point(DEFENSIVE_X_COORD, predBall.y - .2, lookAtPoint)

  def defensiveStrats_augmented(self):
    print("theta = {}, ball vel = {}".format(self.getMyLocation().theta, self.ballVelocity.magnitude()))
    if ((distance(self.getMyLocation(), self.ballLocation) < .1 or self.ballVelocity.magnitude() < .2)
       and self.getMyLocation().x > self.ballLocation.x
       and (self.getMyLocation().theta > math.pi/2 and self.getMyLocation().theta < 3*math.pi / 2)):
      # kick() # xq no lol
      self.getBehindBall_default()
    else:
      print("\nDEFEND AGAIN")
      predBall = self.ballPrediction(1)
      lookAtPoint = self.ballLocation
      if self.twoRobotStrategyEnabled == 1:
        if self.robotAssignment == 1:
          DEFENSIVE_X_COORD = HOME_GOAL.x - 0.2
        else:
          DEFENSIVE_X_COORD = HOME_GOAL.x - 0.8
      else:
        DEFENSIVE_X_COORD = max(self.ballLocation.x, HOME_GOAL.x - 0.3)

      DEFENSIVE_Y_COORD = predBall.y

      # don't leave the goalie box
      DEFENSIVE_Y_COORD = min(DEFENSIVE_Y_COORD, HEIGHT_FIELD_METER/4)
      DEFENSIVE_Y_COORD = max(DEFENSIVE_Y_COORD, -HEIGHT_FIELD_METER/4)

      # only guard if the ball isn't past the robot
      if self.ballLocation.x < self.home1Location.x:
        self.go_to_point(DEFENSIVE_X_COORD, DEFENSIVE_Y_COORD, lookAtPoint)
      else:
        offset = 0.2 if self.getMyLocation().y < 0 else -0.2
        self.go_to_point(DEFENSIVE_X_COORD, predBall.y + offset)

  def play(self):
    if self.playState == PlayState.CHECK:
      self.checkState_default()

    if self.playState == PlayState.RUSHGOAL:
      self.rushGoal_default()

    if self.playState == PlayState.RETURN_TO_PLAY:
      self.returnToPlay_default()

    #check if ball is behind robot
    if self.playState == PlayState.GET_BEHIND_BALL:
      self.getBehindBall_default()

  def returnToPlay_default(self):
    self.go_to_point(CENTER.x, CENTER.y, AWAY_GOAL)
    if abs(self.home1Location.x) < .2 and abs(self.home1Location.y) < .2:
      self.playState = PlayState.CHECK

  def checkState_default(self):
    self.playState = PlayState.GET_BEHIND_BALL
    if self.home1Location.x > pixelToMeter(345):
      self.playState = PlayState.RETURN_TO_PLAY
    elif (MotionSkills.isPointInFrontOfRobot(self.home1Location, self.ballLocation, 0.5, 0.04 + abs(MAX_SPEED/4))): #This offset compensates for the momentum
      print("REALLY BEHIND BALL")
      self.playState = PlayState.RUSHGOAL# rush goal


  def rushGoal_default(self):
    print("RUSHING")
    #self.speed = RUSH_SPEED
    self.go_to_point(AWAY_GOAL.x - .2, AWAY_GOAL.y, Point(AWAY_GOAL.x - .2, AWAY_GOAL.y))
    self.playState = PlayState.GET_BEHIND_BALL
    print("my pos = {}, rush = {}".format(self.getMyLocation().x, AWAY_GOAL.x + 1))
    if self.getMyLocation().x < 100000000: #-START_LOC.x:
      kick()
      print("KICKED at {}".format(getTime()))
      self.stopRushingGoalTime = getTime() + 50 #int(2 * DIS_BEHIND_BALL/MAX_SPEED*100)


  def getBehindBall_default(self):
    predBallLoc = self.ballPrediction(time.time() - self.lastVisionUpdatedTimeStamp)
    desiredPoint = MotionSkills.getPointBehindBallXY(predBallLoc.x, predBallLoc.y, home_goal=Point(AWAY_GOAL.x - 0.2, AWAY_GOAL.y))

    distFromPoint = distance(self.home1Location, desiredPoint)

    desired_theta = math.atan2(AWAY_GOAL.y-self.getMyLocation().y, AWAY_GOAL.x-0.2-self.getMyLocation().x)
    desired_theta = desired_theta % (2*math.pi)
    theta_error = desired_theta - self.getMyLocation().theta
    if abs(theta_error) > math.pi:
      if theta_error > 0:
        theta_error = theta_error - 2*math.pi
      else:
        theta_error = theta_error + 2*math.pi

#    if (distFromPoint < 0.05 or distance(self.ballLocation, self.home1Location) < 0.1) and (abs(error) < math.pi/8) and (self.home1Location.x > self.ballLocation.x):
    if (distFromPoint < 0.05) and (abs(theta_error) < math.pi/8) and (self.home1Location.x > self.ballLocation.x):
      print("CHANGING TO RUSSIAN")
      self.playState = PlayState.RUSHGOAL
      self.stopRushingGoalTime = getTime() + 50
      self.rushGoal_default()
    elif self.ballLocation.x > AWAY_GOAL and (self.home1Location.x < self.ballLocation.x):
      print("IN FRONT OF BALL")
      xoffset = .2
      offset = 0.2 if self.getMyLocation().y < 0 else -0.2
      self.go_to_point(desiredPoint.x - xoffset, desiredPoint.y + offset)
    elif self.ballLocation.x > AWAY_GOAL:
      print("GOING TO ACTUAL PT BEHIND BALL {}, {}".format(meterToPixel(desiredPoint.x), meterToPixel(desiredPoint.y)))
      #try go to point
      self.go_to_point(desiredPoint.x, desiredPoint.y, lookAtPoint=AWAY_GOAL)

  def jarjar_oneRobotStrategy(self):
    #if (robot is on the bottom quarter of our field)
    # goFullDefensive()
    #if (robot is anywhere else)
    # goFullOffensive()
    if (abs(self.ballLocation.x)  > HOME_GOAL.x + .05) and (abs(self.ballLocation.y) < pixelToMeter(67)):
      self.gameState = GameState.START_POSITION
    #elif self.ballLocation.x > WIDTH_FIELD/4 and (self.awayTeam1Location.x > 0 or self.awayTeam2Location.x > 0):
    elif self.ballLocation.x > START_LOC.x:
      self.strategy = Strategy.DEFENSIVE
      self.defensiveStrats_augmented()
    else:
      self.strategy = Strategy.OFFENSIVE
      self.play()

  def jarjar_baeMAXStrategy(self):
    pass

  def play_baeMAX(self):
    if self.playState == PlayState.CHECK:
      self.checkState_default()

    if self.playState == PlayState.RUSHGOAL:
      self.rushGoal_default()

    if self.playState == PlayState.RETURN_TO_PLAY:
      self.returnToPlay_default()

    #check if ball is behind robot
    if self.playState == PlayState.GET_BEHIND_BALL:
      self.getBehindBall_baeMAX()

  def getBehindBall_baeMAX(self):
    predBallLoc = self.ballPrediction(time.time() - self.lastVisionUpdatedTimeStamp)
    desiredPoint = MotionSkills.getPointBehindBallXYDistance(predBallLoc.x, predBallLoc.y, 1, home_goal=AWAY_GOAL)
    distFromPoint = distance(self.home1Location, desiredPoint)
    if distFromPoint < 0.05:
      self.playState = PlayState.RUSHGOAL
      self.stopRushingGoalTime = getTime() + 50
      kick()
    elif self.ballLocation.x > AWAY_GOAL:
      #try go to point
      self.go_to_point(desiredPoint.x, desiredPoint.y)
      #self.go_direction(desiredPoint)

  def jarjar_robot1Strategy(self):
    self.play()
    pass

  def jarjar_robot2Strategy(self):
    self.defensiveStrats()
    pass

  def scoreGoal(self):
    pass
    #1. get behind ball


    #2. get ball into goal

  def reassignRobots(self):
    if self.twoRobotStrategyEnabled == 1:
      if self.robotAssignment == 1:
        if self.distance(self.home2location, self.ballLocation) < .1:
          self.robotAssignment = 2
      else:
        if self.distance(self.home2location, self.ballLocation) < .1:
          self.robotAssignment = 1
    else:
      self.robotAssignment = 1

  def executionLoop(self, scheduler):
    scheduler.enter(0.1, 1, self.executionLoop,(scheduler,))
    #we are the first robot
    self.reassignRobots()
    if self.robotAssignment == 1:
      if self.gameState == GameState.PLAY:
        if self.twoRobotStrategyEnabled == 1:
          self.jarjar_robot1Strategy()
        else:
          self.jarjar_oneRobotStrategy()
      elif self.gameState == GameState.TEST:
        self.defensiveStrats()
        self.strategy = Strategy.DEFENSIVE
      elif self.gameState == GameState.STOP:
        self.strategy = Strategy.NONE
        self.sendCommand(0, 0, 0)
      elif self.gameState == GameState.CENTER:
        self.strategy = Strategy.NONE
        if distance(self.home1Location, Point(CENTER.x, CENTER.y)) > MOVEMENT_THRESHOLD:
          self.go_to_point(CENTER.x, CENTER.y, None)
        else:
          self.sendCommand(0, 0, 0)
      elif self.gameState == GameState.START_POSITION:
        self.strategy = Strategy.NONE
        if distance(self.home1Location, START_LOC) > MOVEMENT_THRESHOLD or abs(self.home1Location.theta) > 0.1:
          self.go_to_point(START_LOC.x, START_LOC.y, AWAY_GOAL)
        else:
          self.sendCommand(0, 0, 0)
      elif self.gameState == GameState.GOTOPOINT:
        self.strategy = Strategy.NONE
        if distance(self.home1Location, self.clickLocation) > MOVEMENT_THRESHOLD:
          self.go_to_point(self.clickLocation.x, self.clickLocation.y)
        else:
          self.sendCommand(0, 0, 0)
      #we are the second robot
      else:
        if self.gameState == GameState.PLAY:
          if self.twoRobotStrategyEnabled == 1:
            self.jarjar_robot2Strategy()
          else:
            self.jarjar_oneRobotStrategy()
        elif self.gameState == GameState.TEST:
          self.defensiveStrats()
          self.strategy = Strategy.DEFENSIVE
        elif self.gameState == GameState.STOP:
          self.strategy = Strategy.NONE
          self.sendCommand(0, 0, 0)
        elif self.gameState == GameState.CENTER:
          self.strategy = Strategy.NONE
          if distance(self.home1Location, Point(CENTER.x, CENTER.y)) > MOVEMENT_THRESHOLD:
            self.go_to_point(CENTER.x, CENTER.y, None)
          else:
            self.sendCommand(0, 0, 0)
        elif self.gameState == GameState.START_POSITION:
          self.strategy = Strategy.NONE
          if distance(self.home1Location, START_LOC) > MOVEMENT_THRESHOLD or abs(self.home1Location.theta) > 0.1:
            self.go_to_point(START_LOC.x, START_LOC.y, AWAY_GOAL)
          else:
            self.sendCommand(0, 0, 0)
        elif self.gameState == GameState.GOTOPOINT:
          self.strategy = Strategy.NONE
          if distance(self.home1Location, self.clickLocation) > MOVEMENT_THRESHOLD:
            self.go_to_point(self.clickLocation.x, self.clickLocation.y)
          else:
            self.sendCommand(0, 0, 0)

  def watchdog(self, scheduler):
    # print("WOOF {}".format(time.time()))
    if time.time() > self.lastVisionUpdatedTimeStamp + 0.5:
      print("WIFI TIMED OUT")
      self.sendCommand(0, 0, 0)
      self.oldGameState = self.gameState
      self.gameState = GameState.STOP
      time.sleep(1)
      self.gameState = self.oldGameState
      scheduler.enter(0.25, 2, self.watchdog, (scheduler,))
    else:
      scheduler.enter(0.25, 2, self.watchdog, (scheduler,))

  def executeCommCenterCommand(self,req):
    self.resetPIDValues()
    if req.comm == 1:
      self.gameState = GameState.STOP
      self.playState = PlayState.CHECK
    elif req.comm == 2:
      self.gameState = GameState.PLAY
    elif req.comm == 3:
      self.gameState = GameState.CENTER
      self.playState = PlayState.CHECK
    elif req.comm == 4:
      self.gameState = GameState.START_POSITION
      self.playState = PlayState.CHECK
    elif req.comm == 5:
      self.gameState = GameState.TEST
    elif req.comm == 6:
      self.gameState = GameState.GOTOPOINT
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
    s.enter(0, 1, self.executionLoop, (s,))
    s.enter(0.5, 2, self.watchdog, (s,))
    s.run()

  def updatePredictions(self):
    # get new timestamp
    newTimeStamp = time.time()
    sample_period = newTimeStamp - self.lastVisionUpdatedTimeStamp
    self.lastVisionUpdatedTimeStamp = newTimeStamp

    # update ball prediction
    ball_vector_x = (self.ballLocation.x - self.lastBallLocation.x) #x distance traveled
    ball_vector_y = (self.ballLocation.y - self.lastBallLocation.y) #y distance traveled
    ball_mag = math.sqrt(ball_vector_x**2 + ball_vector_y**2)
    ball_angle = math.atan2(ball_vector_y, ball_vector_x)
    ball_velocity = ball_mag/sample_period

    self.ballVelocity.x = ball_vector_x / sample_period
    self.ballVelocity.y = ball_vector_y / sample_period

    self.lastBallLocation.x = self.ballLocation.x
    self.lastBallLocation.y = self.ballLocation.y

    # update robot prediction
    home1_vector_x = (self.home1Location.x - self.lastHome1Location.x) #x distance traveled
    home1_vector_y = (self.home1Location.y - self.lastHome1Location.y) #y distance traveled
    home1_mag = math.sqrt(home1_vector_x**2 + home1_vector_y**2)
    home1_angle = math.atan2(home1_vector_y, home1_vector_x)
    home1_velocity = home1_mag/sample_period

    self.home1Velocity.x = home1_vector_x / sample_period
    self.home1Velocity.y = home1_vector_y / sample_period

    self.lastHome1Location.x = self.home1Location.x
    self.lastHome1Location.y = self.home1Location.y

  def ballPrediction(self, time_sec):
    #time_sec is saying where will the ball be in 'time_sec' amount of seconds
    ball_new_position_x = self.ballLocation.x + self.ballVelocity.x*time_sec
    ball_new_position_y = self.ballLocation.y + self.ballVelocity.y*time_sec
    return Point(ball_new_position_x, ball_new_position_y)

  def resetPIDValues(self):
    for key in self.integrator:
      self.integrator[key] = 0
    for key in self.differentiator:
      self.integrator[key] = 0

  def pidloop(self, dest_loc, cur_loc, var):
    def getConstants(var):
      if var == 'x':
        return (0.1, 0.05, 1.1, .13, 0.15, MAX_SPEED)
        # return (0.01, 0.05, 2.3, 0.12, 3.7, MAX_SPEED) #.12, 3.7
      elif var == 'y':
        return (0.1, 0.05, 1.1, .13, 0.23, MAX_SPEED)
        # return (0.01, 0.05, 2.3, 0.12, 3.89, MAX_SPEED) #.12, 3.89
      elif var == 'theta':
        return (0.1, 0.05, 1, .4, 0.25, MAX_OMEGA)
        # return (0.01, 0.05, 1.7, 0, .7, MAX_OMEGA) #.7
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

    #print("PID cur = {}, dest = {}, error = {}".format(cur_loc, dest_loc, error))
    #update integrator

    if abs(self.desired_old[var] - dest_loc) > .07:
      self.integrator[var] = 0

    self.integrator[var] = self.integrator[var] + (Ts/2) * (error + self.error_d1[var])

    #update differentiator
    self.differentiator[var] = ((2*tau - Ts)/(2*tau + Ts))*self.differentiator[var] + (2/(2*tau + Ts))*(error - self.error_d1[var])

    #u is the output velocity in a specified direction (x or y)
    velocity = sat(kp*error + ki*self.integrator[var] + kd*self.differentiator[var], limit)

    if ki != 0:
      velocity_unsat = kp*error + ki*self.integrator[var] + kd*self.differentiator[var]
      #self.integrator[var] = self.integrator[var] + Ts/ki *(velocity - velocity_unsat)

    #if (var == 'x' or var == 'y') and abs(error) < 0.1:
    #  if error < 0:
    #    velocity = -.5
    #  else:
    #    velocity = .5


    f = open('/root/w.txt', 'a')
    if var == 'x':
      f = open('/root/x.txt', 'a')
    elif var == 'y':
      f = open('/root/y.txt', 'a')
    f.write("{},{}\n".format(cur_loc, dest_loc))
    f.close()

    #print("Error d1 = {}".format(self.error_d1[var]))

    self.error_d1[var] = error


    #u is the output velocity in a specified direction (x or y)
    # print("PID", var, error, velocity)

    MIN_SPEED_XY = 0.4
    MIN_SPEED_THETA = 1
    THRESHOLD_XY = 0.1
    THRESHOLD_THETA = 0.5
    self.desired_old[var] = dest_loc

    if math.sqrt(self.home1Velocity.x**2 + self.home1Velocity.y**2) < 0.2:
      if var == 'x' or var == 'y':
        if False and abs(error) < THRESHOLD_XY:
          velocity = 0
        else:
          if velocity > 0:
            velocity += MIN_SPEED_XY #= max(MIN_SPEED_XY, velocity)
          else:
            velocity -= MIN_SPEED_XY #= min(-MIN_SPEED_XY, velocity)
      elif var == 'theta':
        if False and abs(error) < THRESHOLD_THETA:
          velocity = 0
        else:
          if velocity > 0:
            velocity += MIN_SPEED_THETA #= max(MIN_SPEED_THETA, velocity)
          else:
            velocity -= MIN_SPEED_THETA #= min(-MIN_SPEED_THETA, velocity)

    # if math.sqrt(self.home1Velocity.x**2 + self.home1Velocity.y**2) < 0.2:
    #   if var == 'x' or var == 'y':
    #     # print("increase {}".format(var))
    #     if abs(velocity) > THRESHOLD:
    #       velocity = MIN_SPEED_XY if velocity > 0 else -MIN_SPEED_XY
    #   # elif var == 'theta':
    #   #   velocity = velocity + (MIN_SPEED_THETA if velocity > 0 else -MIN_SPEED_THETA)
    return velocity

def distance(cur_loc, dest_loc):
  return math.sqrt((cur_loc.x - dest_loc.x)**2 + (cur_loc.y - dest_loc.y)**2)

if __name__ == '__main__':
  winner = Vektory()
  try:
    os.remove('/root/x.txt')
    os.remove('/root/y.txt')
    os.remove('/root/w.txt')
  except:
    pass
  winner.go()
