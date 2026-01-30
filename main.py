from zed_utils import ZEDBagReader
import cv2

def main():
    reader = ZEDBagReader('c:/Users/scar15/Desktop/zed_rosbag/')
    for frame in reader.images:
        cv2.imshow('RGB', frame)

            
        if cv2.waitKey(33) & 0xFF == ord('q'): # wait 33 key for framerate purpose
            break
            
if __name__ == "__main__":
    main()