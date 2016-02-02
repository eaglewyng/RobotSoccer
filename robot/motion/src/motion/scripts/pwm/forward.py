import roboclaw, time

i = 10
flag = True
while i != -10:
  print(i)
  roboclaw.M2Backward(128,i)
  roboclaw.M1Forward(128,i)
  i = i + 10 if flag else i - 10
  if i > 120:
    i = 120
    flag = False
  time.sleep(0.05)

