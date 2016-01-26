import roboclaw, time

i = 0
flag = True
while True:
  print(i)
  roboclaw.M2Forward(128, i)
  roboclaw.M1Forward(128, i)
  i = i + 10 if flag else i - 10
  if i > 120:
    i = 120
    flag = False
  if i < 0:
    i = 0
    flag = True
  time.sleep(0.05)
