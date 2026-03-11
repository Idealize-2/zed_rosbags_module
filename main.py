from zed_utils import ZEDBagReader
import cv2

BAG_PATH = '/home/walkie/robocup2026/rosbag/2026-03-06-14-36-20'

def main():
    reader = ZEDBagReader(BAG_PATH)
    for time, frame in reader.images:
        cv2.imshow('Image', frame)
        print(f"Time: {time}")

        if cv2.waitKey(1) & 0xFF == ord('q'): # wait 33 key for framerate purpose
            break

if __name__ == "__main__":
    main()
