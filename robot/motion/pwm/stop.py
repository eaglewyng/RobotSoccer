import roboclaw

def stop_all():
  roboclaw.M1Forward(128, 0)
  roboclaw.M1Forward(129, 0)
  roboclaw.M2Forward(128, 0)
