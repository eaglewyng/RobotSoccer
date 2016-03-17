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
from utilities.kick import kick
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

class State(Enum):
  rushGoal = 1
  getBehindBall = 2
  rotateToAngleBehindBall = 3
  check = 4
  returnToPlay = 5

class TestState(Enum):
  check = 1
  rushGoal = 2
  getBehindBall = 3

class Rotate(Enum):
  none = 1
  clockwise = 2
  counterClockwise = 3

class Strategy(Enum):
  DEFENSIVE = 1
  OFFENSIVE = 2
  NONE = 3

class Vektory:
  def __init__(self):
    self.locations = None
    self.ball = Ball()
    self.robotLocation = None
    self.clickLocationX = 0
    self.clickLocationY = 0
    self.distanceToBall = 0
    self.state = State.check
    self.rotate = Rotate.none
    self.stopRushingGoalTime = 0
    self.newCommand = False
    self.vel_x = 0
    self.vel_y = 0
    self.omega = 0
    self.gameState = GameState.stop
    self.stopped = True
    self.testState = TestState.check

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
    command = RobotCommand(-1,vel_x, vel_y, omega, theta)
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
      self.state = State.check
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

  def go_direction(self, point):
    angle = MotionSkills.angleBetweenPoints(self.robotLocation,point)
    self.vel_x = math.cos(angle) * MAX_SPEED
    self.vel_y = math.sin(angle) * MAX_SPEED
    des_angle = MotionSkills.angleBetweenPoints(self.ball.point,HOME_GOAL)
    delta_angle = MotionSkills.deltaBetweenAngles(self.robotLocation.theta,des_angle)
    if abs(delta_angle) < .1:
      self.omega = 0
    else:
      self.omega = delta_angle
    self.newCommand = True

  def go_direction2(self, point):
    angle = MotionSkills.angleBetweenPoints(self.robotLocation,point)
    vel_x = math.cos(angle) * MAX_SPEED
    vel_y = math.sin(angle) * MAX_SPEED
    des_angle = MotionSkills.angleBetweenPoints(self.ball.point,HOME_GOAL)
    delta_angle = MotionSkills.deltaBetweenAngles(self.robotLocation.theta,des_angle)
    if abs(delta_angle) < .1:
      omega = 0
    else:
      omega = delta_angle * 3.0
    self.sendCommand(vel_x,vel_y,omega,self.robotLocation.theta)


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

    #print "go_to_point"
    if lookAtPoint == None:
      lookAtPoint = self.ball.point
    desired_x = x
    desired_y = y

    print("desired (x, y) = ({}, {})").format(desired_x, desired_y)
    print("my pos (x, y, t) = ({}, {}, {})").format(self.robotLocation.x, self.robotLocation.y, self.robotLocation.theta)

    vektor_x = (desired_x-self.robotLocation.x) * SCALE_VEL
    vektor_y = (desired_y-self.robotLocation.y) * SCALE_VEL

    mag = math.sqrt(vektor_x**2+vektor_y**2)
    angle = math.atan2(lookAtPoint.y-self.robotLocation.y, lookAtPoint.x-self.robotLocation.x)

    delta_angle = angle-self.robotLocation.theta
    print("delta angle = {}").format(delta_angle*180/math.pi)

    bestDelta = math.atan2(math.sin(delta_angle), math.cos(delta_angle)) * SCALE_OMEGA
    #print bestDelta
    if mag >= MAX_SPEED:
      vektor_x = (MAX_SPEED/mag)*vektor_x
      vektor_y = (MAX_SPEED/mag)*vektor_y
    elif mag < MIN_SPEED:
      vektor_x = 0
      vektor_y = 0

    # print("WAT {}".format(bestDelta))

    if abs(bestDelta) > MAX_OMEGA:
      bestDelta = MAX_OMEGA
    elif abs(bestDelta) < MIN_OMEGA:
      bestDelta = 0
    bestDelta = 0

    # print("world vel (x, y, w, t) = ({}, {}, {}, {})").format(vektor_x, vektor_y, bestDelta, self.robotLocation.theta)

    self.sendCommand(vektor_x, vektor_y, bestDelta, self.robotLocation.theta)

  def go_to_point_maxspeed(self,x, y, lookAtPoint=None):
    #print "go_to_point"
    if lookAtPoint == None:
      lookAtPoint = self.ball.point
    desired_x = x
    desired_y = y

    print("desired (x, y) = ({}, {})").format(desired_x, desired_y)
    print("my pos (x, y, t) = ({}, {}, {})").format(self.robotLocation.x, self.robotLocation.y, self.robotLocation.theta)

    vektor_x = (desired_x-self.robotLocation.x) * SCALE_VEL
    vektor_y = (desired_y-self.robotLocation.y) * SCALE_VEL

    mag = math.sqrt(vektor_x**2+vektor_y**2)
    angle = math.atan2(lookAtPoint.y-self.robotLocation.y, lookAtPoint.x-self.robotLocation.x)

    delta_angle = angle-self.robotLocation.theta
    print("delta angle = {}").format(delta_angle*180/math.pi)

    bestDelta = math.atan2(math.sin(delta_angle), math.cos(delta_angle)) * SCALE_OMEGA
    #print bestDelta
    vektor_x = (MAX_SPEED/mag)*vektor_x
    vektor_y = (MAX_SPEED/mag)*vektor_y
    #if mag >= MAX_SPEED:
    #  vektor_x = (MAX_SPEED/mag)*vektor_x
    #  vektor_y = (MAX_SPEED/mag)*vektor_y
    #elif mag < MIN_SPEED:
    #  vektor_x = 0
    #  vektor_y = 0

    if abs(bestDelta) > MAX_OMEGA:
      bestDelta = MAX_OMEGA
    elif abs(bestDelta) < MIN_OMEGA:
      bestDelta = 0
    bestDelta = 0

    # print("world vel (x, y, w, t) = ({}, {}, {}, {})").format(vektor_x, vektor_y, bestDelta, self.robotLocation.theta)

    self.sendCommand(vektor_x, vektor_y, bestDelta, self.robotLocation.theta)

  def updateLocations(self, data):
    try:
          # locations = rospy.ServiceProxy('locations', curlocs, persistent=True)
          # response = locations()
          #self.locations = pickle.loads(response.pickle)
        self.locations = Locations()
        self.locations.setLocationsFromMeasurement(data)
        self.robotLocation = self.locations.home1
        self.ball.point.x = self.locations.ball.x
        self.ball.point.y = self.locations.ball.y
        self.distanceToBall = math.sqrt((self.robotLocation.x-self.locations.ball.x)**2+(self.robotLocation.y-self.locations.ball.y)**2)
        #print 'time: %f x: %f  y: %f  theta: %f' %(robotLocation.timestamp, robotLocation.x, robotLocation.y, robotLocation.theta)
        #update predictions
        self.updatePredictions()
        # print("{0:.2f}".format(time.time()))
        # self.last = time.time()

    except rospy.ServiceException, e:
        print "service call failed: %s"%e

  def commandRoboclaws(self):
    print("cmd robo: ({}, {}, {}, {})").format(self.vel_x, self.vel_y, self.omega, self.robotLocation.theta)
    velchange.goXYOmegaTheta(self.vel_x,self.vel_y,self.omega,self.robotLocation.theta)

  def defensiveStrats(self):
    predBallXY = self.ballPrediction(1.5)	#TODO change this to be parameterizable somehow
    lookAtPoint = self.ball.point
    DEFENSIVE_X_COORD = HOME_GOAL.x - 0.2
    # DEFENSIVE_Y_COORD = self.ball.point.y
    DEFENSIVE_Y_COORD = predBallXY[1]

    DEFENSIVE_Y_COORD = min(DEFENSIVE_Y_COORD, HEIGHT_FIELD_METER/4)
    DEFENSIVE_Y_COORD = max(DEFENSIVE_Y_COORD, -HEIGHT_FIELD_METER/4)

    if self.ball.point.x < self.robotLocation.x:
      self.go_to_point(DEFENSIVE_X_COORD, DEFENSIVE_Y_COORD, lookAtPoint)
    else:
      self.go_to_point(DEFENSIVE_X_COORD, HOME_GOAL.y, lookAtPoint)


  def old(self):
    if self.testState == TestState.check:
      if self.robotLocation.x > (self.ball.point.x-DIS_BEHIND_BALL):
        self.testState = TestState.getBehindBall
      else:
        self.testState = TestState.rushGoal
        self.stopRushingGoalTime = getTime() + 100

    if self.testState == TestState.getBehindBall:
      point = Point(self.ball.point.x-DIS_BEHIND_BALL, self.ball.point.y)
      if self.robotLocation.y > self.ball.point.y:
        point.y = point.y + DIS_BEHIND_BALL
      else:
        point.y = point.y - DIS_BEHIND_BALL
      self.go_direction2(point)
      self.testState = TestState.check

    if self.testState == TestState.rushGoal:
      if self.ball.point.y < -.25 and self.robotLocation.y > (self.ball.point.y + DIS_BEHIND_BALL):
        point = Point(self.ball.point.x,self.ball.point.y - 1.0)
        self.go_direction2(point)
      elif self.ball.point.y > .25 and self.robotLocation.y < (self.ball.point.y - DIS_BEHIND_BALL):
        point = Point(self.ball.point.x,self.ball.point.y + 1.0)
        self.go_direction2(point)
      elif abs(self.distanceToBall) > (DIS_BEHIND_BALL*3/4):
        self.go_direction2(self.ball.point)
      else:
        self.go_direction2(HOME_GOAL)
      if getTime() >= self.stopRushingGoalTime:
        self.testState = TestState.check

  def test(self):
    global HOME_GOAL
    HOME_GOAL = Point(HOME_GOAL.x,0)

    # Kick the ball off of the sides when it is too close to the side
    if self.ball.point.y < (MARGIN - HEIGHT_FIELD_METER/2):
      HOME_GOAL = Point(HOME_GOAL.x, -HEIGHT_FIELD_METER)
    if self.ball.point.y > (HEIGHT_FIELD_METER/2 - MARGIN):
      HOME_GOAL = Point(HOME_GOAL.x, HEIGHT_FIELD_METER)

    # Make sure the robot is behind the ball and facing the goal
    if self.testState == TestState.check:
      self.testState = TestState.getBehindBall
      angleBallGoal = MotionSkills.angleBetweenPoints(self.ball.point,HOME_GOAL)
      deltaAngle = MotionSkills.deltaBetweenAngles(self.robotLocation.theta,angleBallGoal)
      if MotionSkills.isPointInFrontOfRobot(self.robotLocation,self.ball.point) and abs(deltaAngle) < .12:
        self.testState = TestState.rushGoal
        self.stopRushingGoalTime = getTime() + 45

    # Get behind the ball. Move to the side first is necessary
    if self.testState == TestState.getBehindBall:
      if (self.ball.point.x + .05) < (self.robotLocation.x):
          point = Point(self.ball.point.x)
          if self.ball.point.y < self.robotLocation.y:
              point.y = self.ball.point.y + DIS_BEHIND_BALL
          else:
              point.y = self.ball.point.y - DIS_BEHIND_BALL
          #Move to the side of the ball
          self.go_to_point(point.x,point.y)
      #Go to behind ball
      else:
          behindTheBallPoint = MotionSkills.getPointBehindBall(self.ball,HOME_GOAL)
          self.go_to_point(behindTheBallPoint.x,behindTheBallPoint.y)
      self.testState = TestState.check

    # Rush the goal and kick for the specified time
    if self.testState == TestState.rushGoal:
      self.go_to_point(HOME_GOAL.x,HOME_GOAL.y,HOME_GOAL)
      if self.distanceToBall < .05:
        kick()
      if self.stopRushingGoalTime <= getTime():
        self.testState = TestState.check

  def play(self):
    # self.commandRoboclaws()
    #print (self.robotLocation.x, self.robotLocation.y)
    #self.setSpeed()
    #check if robot is ready to rush goal
    if self.state == State.check:
      self.state = State.getBehindBall
      if self.robotLocation.x > pixelToMeter(345):
        self.state = State.returnToPlay
      elif (MotionSkills.isPointInFrontOfRobot(self.robotLocation,self.ball.point, 0.5, 0.04 + abs(MAX_SPEED/4))): #This offset compensates for the momentum
        print("REALLY BEHIND BALL")
        self.state = State.rushGoal# rush goal
        self.stopRushingGoalTime = getTime() + int(2 * DIS_BEHIND_BALL/MAX_SPEED*100)

    if self.state == State.rushGoal:
      print("RUSHING")
      #self.speed = RUSH_SPEED
      self.go_to_point(AWAY_GOAL.x, AWAY_GOAL.y, AWAY_GOAL)
      if getTime() >= self.stopRushingGoalTime:
        kick()
        print("KICKED")
        self.state = State.check

    if self.state == State.returnToPlay:
      self.go_to_point(CENTER.x, CENTER.y, AWAY_GOAL)
      if abs(self.robotLocation.x) < .2 and abs(self.robotLocation.y) < .2:
        self.state = State.check

    #check if ball is behind robot
    if self.state == State.getBehindBall:
      predBallLoc = this.ballPrediction(time.time() - self.lastTimeStamp)
      desiredPoint = MotionSkills.getPointBehindBallXY(predBallLoc(0), predBallLoc(1), home_goal=AWAY_GOAL)
      distFromPoint = math.sqrt((self.robotLocation.x - desiredPoint.x)**2
                              + (self.robotLocation.y - desiredPoint.y)**2)
      if distFromPoint < 0.1:
        self.state = State.rushGoal
        self.stopRushingGoalTime = getTime() + 50
      elif self.ball.point.x > AWAY_GOAL:
        #try go to point
        self.go_to_point(desiredPoint.x, desiredPoint.y)
        #self.go_direction(desiredPoint)

      # print("GETTING BEHIND BALL")
      # #robot in front of ball
      # if MotionSkills.isBallBehindRobot(self.robotLocation, self.ball.point):
      #   print("DONE GOOFED")
      #   # This gets a point beside the ball perpendicular to the line of the ball and the goal
      #   #point = getPointBesideBall(self.robotLocation, self.ball.point, DIS_BEHIND_BALL)

      #   point = Point(self.ball.point.x)
      #   # if robot above ball
      #   if self.ball.point.y < self.robotLocation.y:
      #       point.y = self.ball.point.y + DIS_BEHIND_BALL
      #   else:
      #       point.y = self.ball.point.y - DIS_BEHIND_BALL

      #   self.go_direction(point)
      #   #Move to the side of the ball
      #   #self.go_to_point(point.x,point.y)
      # #robot behind ball
      # else:
      #   print("GOT BEHIND BALL")
      #   behindTheBallPoint = MotionSkills.getPointBehindBall(self.ball)
      #   self.go_direction(behindTheBallPoint)
      #   self.state = State.check

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
    self.commandRoboclaws()

    #1. get behind ball


    #2. get ball into goal

  def executionLoop(self, scheduler):
    scheduler.enter(0.1, 1, self.executionLoop,(scheduler,))
    if self.gameState == GameState.play:
      self.jarjar_oneRobotStrategy()
    elif self.gameState == GameState.test:
      #self.test()
      self.defensiveStrats()
      self.strategy = Strategy.DEFENSIVE
    elif self.gameState == GameState.stop:
      self.strategy = Strategy.NONE
      if self.stopped == False:
        self.sendCommand(0,0,0);
        self.stopped = True;
    elif self.gameState == GameState.center:
      self.strategy = Strategy.NONE
      if abs(self.robotLocation.x) > CENTER_THRESHOLD or abs(self.robotLocation.y) > CENTER_THRESHOLD:
        self.go_to_point(CENTER.x, CENTER.y, None)
      elif self.stopped == False:
        self.sendCommand(0,0,0);
        self.stopped = True;
    elif self.gameState == GameState.startPosition:
      self.strategy = Strategy.NONE
      if abs(self.robotLocation.x - pixelToMeter(-120)) > .1 or abs(self.robotLocation.y - 0) > .1 or abs(self.robotLocation.theta) > .1:
        self.go_to_point(pixelToMeter(115), 0, AWAY_GOAL)
      elif self.stopped == False:
        self.sendCommand(0,0,0);
        self.stopped = True;
    elif self.gameState == GameState.goToPoint:
      self.strategy = Strategy.NONE
      if abs(self.robotLocation.x - self.clickLocationX) > CENTER_THRESHOLD/5 or abs(self.robotLocation.y - self.clickLocationY) > CENTER_THRESHOLD/5:
        self.go_to_point(self.clickLocationX, self.clickLocationY)
      elif self.stopped == False:
        self.sendCommand(0, 0, 0)
        self.stopped = True
      # if abs(self.robotLocation.x - ) > CENTER_THRESHOLD or abs(self.robotLocation.y) > CENTER_THRESHOLD:
      #   self.go_to_point(CENTER.x, CENTER.y, None)
      # elif self.stopped == False:
      #   self.sendCommand(0,0,0);
      #   self.stopped = True;


  def executeCommCenterCommand(self,req):
    self.resetPIDState()
    if req.comm == 1:
      self.gameState = GameState.stop
      self.state = State.check
      self.stopped = False
    elif req.comm == 2:
      self.gameState = GameState.play
    elif req.comm == 3:
      self.gameState = GameState.center
      self.state = State.check
      self.stopped = False
    elif req.comm == 4:
      self.gameState = GameState.startPosition
      self.state = State.check
      self.stopped = False
    elif req.comm == 5:
      self.testState = TestState.check
      self.gameState = GameState.test
    elif req.comm == 6:
      self.gameState = GameState.goToPoint
      self.stopped = False
      self.clickLocationX = pixelToMeter(req.x)
      self.clickLocationY = pixelToMeter(req.y)

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
    return (ball_new_position_x, ball_new_position_y)

  def resetPIDState(self):
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

if __name__ == '__main__':
  winner = Vektory()
  winner.go()
