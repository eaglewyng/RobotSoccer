import cv2
import urllib
import numpy as np

stream_path = "http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg"

# boundary values are in reverse RGB order (BGR)
boundaries = [
    ([0  , 0  , 75 ], [100, 100, 255]),              # red
    ([0  , 75 , 0  ], [120, 255, 120]),              # green
    ([75 , 0  , 0  ], [255, 255, 50 ]),              # blue
]

def main():
	stream = urllib.urlopen(stream_path)

	cv2.namedWindow("images", flags = cv2.WINDOW_NORMAL)
	cv2.resizeWindow("images", 400, 1000)

	bytes = ''
	while True:
		bytes += stream.read(1024)
		start = bytes.find('\xff\xd8')
		end = bytes.find('\xff\xd9')

		if start != -1 and end != -1:
			jpg = bytes[start:end+2]
			bytes = bytes[end+2:]
			image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

			image = cv2.resize(image, (0, 0), fx=0.6, fy=0.6)
			output = [None for x in boundaries]
			for i, (lower, upper) in enumerate(boundaries):
			    lower = np.array(lower, dtype = "uint8")
			    upper = np.array(upper, dtype = "uint8")

			    mask = cv2.inRange(image, lower, upper)
			    output[i] = cv2.bitwise_and(image, image, mask = mask)

			images = [image] + output
			cv2.imshow('images', np.vstack(images))
			cv2.imwrite("images.jpg", image)
			if cv2.waitKey(1) == 27:
				break

if __name__ == "__main__":
	main()