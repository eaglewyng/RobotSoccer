# import the necessary packages
import numpy as np
import argparse
import cv2
import time

boundaries = [
    ([0  , 0  , 75 ], [100, 100, 255]),              # red
    ([0  , 75 , 0  ], [120, 255, 120]),              # green
    ([75 , 0  , 0  ], [255, 255, 50 ]),              # blue
]

cap = cv2.VideoCapture(0)
cv2.namedWindow("images", flags = cv2.WINDOW_NORMAL)
cv2.resizeWindow("images", 400, 1000)

output = [None for x in boundaries]

while(True):
    ret, image = cap.read()

    image = cv2.resize(image, (0, 0), fx=0.6, fy=0.6)
    for i, (lower, upper) in enumerate(boundaries):
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(image, lower, upper)
        output[i] = cv2.bitwise_and(image, image, mask = mask)

    images = [image] + output
    cv2.imshow("images", np.vstack(images))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()