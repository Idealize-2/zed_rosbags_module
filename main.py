from zed_utils import ZEDBagReader
import cv2

def main():
    reader = ZEDBagReader('/home/walkie/robocup2026/rosbag/13022026-SImulation')
    for frame in reader.point_clouds:
        cv2.imshow('PointCloud', frame)
        #print(frame)

            
        if cv2.waitKey(33) & 0xFF == ord('q'): # wait 33 key for framerate purpose
            break
            
if __name__ == "__main__":
    main()