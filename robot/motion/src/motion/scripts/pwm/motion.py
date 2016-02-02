import roboclaw, time, math

def rampOnce(dirFunc):
    up = True
    i = 10
    while i >= 0:
        print(i)
        dirFunc(i)
        i = i + 10 if up else i - 10
        if i > 120:
          i = 120
          up = False
        time.sleep(0.05)

def forward(i):
    roboclaw.M2Backward(128, i)
    roboclaw.M1Forward(128, i)

def left(i):
    roboclaw.M2Backward(128, int(round(i/math.sqrt(3.0))))
    roboclaw.M1Backward(128, int(round(i/math.sqrt(3.0))))
    roboclaw.M1Forward(129, i)

def right(i):
    roboclaw.M2Forward(128, int(round(i/math.sqrt(3.0))))
    roboclaw.M1Forward(128, int(round(i/math.sqrt(3.0))))
    roboclaw.M1Backward(129, i)

def backward(i):
    roboclaw.M2Forward(128, i)
    roboclaw.M1Backward(128, i)
