import cv2

def __gstreamer_pipeline(
        camera_id,
        capture_width=1920,
        capture_height=1080,
        display_width=1920,
        display_height=1080,
        framerate=30,
        flip_method=0,
    ):
    return (
            "v4l2src device=/dev/video%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)MJPG, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True"
            % (
                    camera_id,
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height,
            )
    )
   
# stream = cv2.VideoCapture(__gstreamer_pipeline(camera_id=0, flip_method=2), cv2.CAP_GSTREAMER)

# stream = cv2.VideoCapture("gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! decodebin ! autovideosink")
# stream = cv2.VideoCapture("v4l2src device=/dev/video0 ! video/x-h264,format=NV12,width=1920,height=1080,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert ! appsink")
stream = cv2.VideoCapture("v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! decodebin ! appsink")

import time

time.sleep(2.0)

ret, frame = stream.read()

if ret:
    print("Frame read successfully")
else:
    print("Failed to read frame")

stream.release()
