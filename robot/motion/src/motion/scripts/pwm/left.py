import roboclaw, time, math

i = 10
flag = True
while i != -10:
  print(i)
  roboclaw.M2Backward(128,int(round(i/math.sqrt(3.0))))
  roboclaw.M1Backward(128,int(round(i/math.sqrt(3.0))))
  roboclaw.M1Forward(129,i)
  i = i + 10 if flag else i - 10
  if i > 120:
    i = 120
    flag = False
  if i < 0:
    i = 0
    #flag = True
  time.sleep(0.05)

