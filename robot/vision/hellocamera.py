# import numpy as np
# import cv2

# cap = cv2.VideoCapture(0)

# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#     # Our operations on the frame come here
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Display the resulting frame
#     cv2.imshow('frame',gray)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()



# import the necessary packages
import numpy as np
import argparse
import cv2

cap = cv2.VideoCapture(0)

lower_blue = [33, 71, 119]
upper_blue = [64, 127, 177]

# define the list of boundaries
# ([10, 150, 110], [20, 190, 190]),         # blue
boundaries = [
    ([110, 65, 30], [180, 130, 70]),
    # ([86, 31, 4], [220, 88, 50]),
    # ([17, 15, 100], [50, 56, 200]),
    # ([25, 146, 190], [62, 174, 250]),
    # ([103, 86, 65], [145, 133, 128])
]

cv2.namedWindow("images", flags = cv2.WINDOW_NORMAL)
cv2.resizeWindow("images", 800, 600)

while(True):
    ret, image = cap.read()
    image = cv2.resize(image, (0, 0), fx=0.4, fy=0.4)
    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        # find the colors within the specified boundaries and apply the mask
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)

        # show the images
        cv2.imshow("images", np.vstack([image, output]))
        # cv2.waitKey(0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()