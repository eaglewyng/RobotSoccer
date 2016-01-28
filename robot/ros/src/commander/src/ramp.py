import roboclaw, time

def singleramp():
  i = 0
  j = 0
  flag = True
  while j < 1:
    print(i)
    roboclaw.M1Forward(128, i)
    roboclaw.M1Forward(129, i)
    roboclaw.M2Forward(128, i)
    i = i + 10 if flag else i - 10
    if i > 120:
      i = 120
      flag = False
    if i < 0:
      i = 0
      flag = True
      j = 1
    time.sleep(0.05)
