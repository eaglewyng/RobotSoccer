import cv2
import urllib
import numpy as np

stream_path = "http://192.168.1.10:8080/stream?topic=/image&dummy=param.mjpg"

# boundary values are in reverse RGB order (BGR)
boundaries = [
    # ([0  , 0  , 75 ], [100, 100, 255]),              # red
    # ([0  , 75 , 0  ], [120, 255, 120]),              # green
    ([75 , 100  , 100 ], [130, 255, 255 ]),              # blue
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
			image_rgb = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
			image_hsv = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2HSV)

			output = [None for x in boundaries]
			for i, (lower, upper) in enumerate(boundaries):
			    lower = np.array(lower, dtype = "uint8")
			    upper = np.array(upper, dtype = "uint8")

			    mask = cv2.inRange(image_hsv, lower, upper)
			    result = cv2.bitwise_and(image_hsv, image_hsv, mask = mask)
			    output[i] = cv2.cvtColor(result, cv2.COLOR_HSV2BGR)

			images = [image_rgb] + output
			cv2.imshow('images', np.vstack(images))
			cv2.imwrite("images.jpg", image_rgb)
			if cv2.waitKey(1) == 27:
				break

if __name__ == "__main__":
	main()