import cv2
import time

# Define resolutions
resolutions = [(640, 480), 
               (1280, 720), 
               (1920, 1080), 
               (3264, 2448),
               (4208, 3120),
               (4656, 3496)]

# resolutions = [(3264, 2448)]

# Initialize camera capture object
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

# Loop through resolutions and capture images
for res in resolutions:
    # Set camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
    cap.set(cv2.CAP_PROP_MODE, cv2.VideoWriter_fourcc(*"MJPG"))
    
    time.sleep(1)

    print(f'Testing Resolution: {res}')

    # Capture image
    ret, frame = cap.read()
    
    # cv2.addText(frame, "Hello", (100, 100), "Arial", 10, (255, 255, 255))

    # Save image to file
    filename = f"{res[0]}x{res[1]}.jpg"
    cv2.imwrite(filename, frame)
    
# Release camera capture object
cap.release()
