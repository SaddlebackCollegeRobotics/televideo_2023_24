import cv2
import numpy


def show_image(window_name: str, window_size: tuple[int, int], frame):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, window_size)
    cv2.imshow(window_name, frame)

def capture_image(camera_index: int, resolution: tuple[int, int]):

    # Initialize camera capture object
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)

    # Camera config.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    cap.set(cv2.CAP_PROP_MODE, cv2.VideoWriter_fourcc(*"MJPG"))

    # Capture image
    ret, frame = cap.read()
    
    cap.release()

    return frame


def main():

    frame1 = capture_image(camera_index=2, resolution=(4656, 3496))
    frame2 = capture_image(camera_index=4, resolution=(4656, 3496))

    if frame1 is not None and frame2 is not None:
        resized_frame_1 = cv2.resize(frame1, (960, 1080))
        resized_frame_2 = cv2.resize(frame2, (960, 1080))

        combined_frame = numpy.concatenate((resized_frame_1, resized_frame_2), axis=1)

        show_image("output", (1920, 1080), combined_frame)

        while True:
            cv2.waitKey(1)

    else:
        print("One or more frames is None")




if __name__ == "__main__":
    main()

