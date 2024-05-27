import cv2
import numpy as np
from time import sleep

FPS = 30
CAP_RESOLUTION = (1920, 1080)
# CAP_RESOLUTION = (4656, 3496)
WINDOW_RESOLUTION = (1920, 1080)

cap = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)
# cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_RESOLUTION[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_RESOLUTION[1])
cap.set(cv2.CAP_PROP_FPS, FPS)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_BUFFERSIZE, 10)

# cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
# cap.set(cv2.CAP_PROP_EXPOSURE , -20)
# cap.set(cv2.CAP_PROP_SATURATION, 75)
# cap.set(cv2.CAP_PROP_CONTRAST, 75)
# cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
# cap.set(cv2.CAP_PROP_GAIN, 75)



cv2.namedWindow('frame', cv2.WINDOW_NORMAL)

sharpen_filter = np.array([[-1,-1,-1],
                         [-1,9,-1],
                         [-1,-1,-1]
                        ])

sharpen_filter2 = np.array([[0,-1,0],
                         [-1,5,-1],
                         [0,-1,0]
                        ])

while True:

    ret, frame = cap.read()

    if ret == True:

        # frame = cv2.filter2D(frame, -1, sharpen_filter2)
        # frame = cv2.GaussianBlur(frame, (5, 5), 0)

 
        # remove noise from image
        # frame = cv2.fastNlMeansDenoisingColored(frame, None, 10, 10, 5, 5)
        
        # rows, cols
        # crop = frame[0:CAP_RESOLUTION[1], 0:CAP_RESOLUTION[0]] 
        cv2.resizeWindow('frame', WINDOW_RESOLUTION)
        cv2.imshow('frame', frame)

        # sleep(1/FPS)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()