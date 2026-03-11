import cv2
import open3d as o3d
import numpy as np
from zed_utils import ZEDBagReader

BAG_PATH = '/home/walkie/robocup2026/rosbag/walkie-rosbag/walkie-simulation-gazebo-01-06032026'
CHUNK_SIZE = 50  # Load 50 frames into memory at a time

def display_zed_bag(bag_path_str):
    reader = ZEDBagReader(bag_path_str)
    
    # Initialize Open3D Visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="ZED 3D Point Cloud", width=800, height=600)
    pcd = o3d.geometry.PointCloud()
    
    # Add a coordinate frame for orientation
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(axis)
    
    first_pcd = True
    stop_playback = False

    print(f"Loading bag in chunks of {CHUNK_SIZE} frames...")

    # Iterate through chunks dynamically
    for chunk_idx, chunk in enumerate(reader.read_chunks(chunk_size=CHUNK_SIZE)):
        if stop_playback:
            break
            
        print(f"Processing chunk {chunk_idx + 1} ({len(chunk)} frames)")
        
        # Iterate through frames inside the current chunk
        for frame in chunk:
            img_time = frame["time"]
            
            # --- Display RGB ---
            cv2.imshow('ZED RGB', frame["image"])

            # Print Pose
            if frame["pose"] is not None:
                pose_time, pos, quat = frame["pose"]
                print(f"[{img_time:.2f}] Position: {pos}")

            # --- Display Depth ---
            if frame["depth"] is not None:
                depth_time, depth_img = frame["depth"]
                depth_img = np.where(np.isfinite(depth_img), depth_img, 10.0)
                depth_viz = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
                cv2.imshow('ZED Depth', depth_viz)

            # --- Display Point Cloud ---
            if frame["point_cloud"] is not None:
                pc_time, xyz, rgb = frame["point_cloud"]
                
                pcd.points = o3d.utility.Vector3dVector(xyz)
                if rgb is not None:
                    pcd.colors = o3d.utility.Vector3dVector(rgb)

                if first_pcd:
                    vis.add_geometry(pcd)
                    first_pcd = False
            
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            # Wait key for framerate and quit command
            key = cv2.waitKey(33) & 0xFF
            if key == ord('q'):
                stop_playback = True
                break

    vis.destroy_window()
    cv2.destroyAllWindows()
    print("Playback finished.")

if __name__ == "__main__":
    display_zed_bag(BAG_PATH)