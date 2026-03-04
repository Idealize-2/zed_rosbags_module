from zed_utils import ZEDBagReader
import cv2

def main():
    reader = ZEDBagReader('/home/walkie/walkie-projects/walkie-ros-ws/rosbag/2026-03-04-18-07-59-Sim')
    for time, frame in reader.images:
        cv2.imshow('Image', frame)
        print(f"Time: {time}")


            
        if cv2.waitKey(1) & 0xFF == ord('q'): # wait 33 key for framerate purpose
            break
            
if __name__ == "__main__":
    main()