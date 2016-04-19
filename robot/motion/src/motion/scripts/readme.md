#Main code folder
##File descriptions

###Vektory.py
`Vektory.py` contains the code that is run in a loop on the robot. It has all of the robot's AI strategies, 
and subscribes to `locTopic` (which is provided by the vision code) and listens to the commcenter service, which
sends the keys pressed by the commandcenter to the robot.

We apologize in advance for the clutter found in `Vektory.py`; it is caused in part by the code we inherited from 
Team Vektor Krum (2015) and the many rushed coding sessions we had. A brief explanation is as follows:

1. The `Vektory` init function initializes all states and the kicking mechanism
2. `go(self)` is called after `Vektory` is initialized, waits for all of the services it needs, and then schedules a run
through its main execution loop `executionLoop()`
3. `executionLoop` checks which robot we're assigned to be (the default is robot 1 unless the parameter `robot` is passed in,
and a two robot strategy will not be enabled unless `self.twoRobotStrategyEnabled` is set to 1. PLEASE NOTE THAT WE NEVER TESTED
OUR TWO ROBOT STRATEGY due to the fact that we did not have enough parts to construct a second robot, so please use it with care).
It then chooses which function to run based on which command was sent by VektoryCommandCenter last.
4. `go_to_point` simply goes to a point while looking at the point specified by `lookAtPoint` (by default, the ball)
5. `getBehindBall_default` attempts to get behind the ball (where "behind the ball" means the ball is between the robot and the away team
goal) and then rushes the goal once it is in a good location. When in front of the ball, it will attempt to avoid the ball while
trying to get behind it.
6. `defensiveStrats_augmented` will guard the goal. It predicts where the ball will be in the future by calling `self.ballPrediction(int)`
if the ball is moving over a certain magnitude, and if the ball is slower than the threshold it will call `getBehindBall_default`
to attempt to get the ball on the other side (and also to prevent stalemates in the case that the other robot is not going after
the ball). 
7. `pidloop` is where the motion control happens. I (@eaglewyng) wish I could tell you a bit more about it, but motion control is
not my expertise. It uses PID controls for x, y, and theta depending on which `var` is passed in, and did a rather good job in our 
competitions (although it was sometimes fidgity). It could probably use some improvement to make it more precise.

There is also a `watchdog` function that will tell the robot to stop if new location information has not been received within a half
second. This is to prevent the robot from running into the wall in the event of a communications failure or perhaps damaging the 
competing robots. The message `WIFI TIMED OUT` should be printed whenever this happens.

###VektoryCommandCenter.py
`VektoryCommandCenter.py` is run by the vision machine and sends any keypresses entered on the command screen (which shows the field)
to `Vektory.py`. Currently, the following commands are implemented:

1. `p` causes the robot to go to the starting position
2. `t` runs the defense strategy
3. `g` runs the real gameplay strategy (where one robot is attacking and defending based on the ball's location)
4. Right clicking anywhere on the field will cause the robot to call go_to_point and attempt to get to that point on the field while facing the ball. 
5. `s` stops all robot movement
6. `c`causes the robot to go to the center
7. `v` switches the cameras from "home" to "away" and vice versa

###Param.py
`Param.py` includes many important parameters that are used in other places in our code, including `RADIUS_ROBOT`, 
`HOME_GOAL`, and `WIDTH_FIELD`. Please make sure the variables are in the form you want them to be in before you
use them, as some are in pixels and others are in meters. For reference, `Vektory` uses almost purely meter variables.

Some refactoring should probably be done in `Vektory.py` and `Param.py` to make sure that changes in `Param.py` will 
change all applicable functions in `Vektory.py`, because in our rush to get our robot competing we probably introduced
some variables in `Vektory.py` that did the exact things that others in `Param.py` did. I will try to change these if I
can but right now our robot is now dismantled, I am moving to another state, and will not be able to test any code I write. 
Therefore, it might be better if you, the student reading this, did the refactoring so I don't break anything.

#Contact
If you have any questions about this code, please contact me at `jarjar_robotsoccer THE_AT_SIGN eaglewyng.com` and I'll try to help or get one of my team members who knows more about your problem to help.
