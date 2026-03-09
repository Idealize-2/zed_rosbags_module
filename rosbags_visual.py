import cv2
import open3d as o3d
import numpy as np
from zed_utils import ZEDBagReader  # Importing your custom module

BAG_PATH = '/home/walkie/robocup2026/rosbag/13022026-SImulation'

def display_zed_bag(bag_path_str):
    # 1. Initialize your ZEDBagReader module
    # This will load all images, depths, and point clouds into memory
    reader = ZEDBagReader(bag_path_str)
    print(len(reader.point_clouds),len(reader.images),len(reader.depths), len(reader.poses))
    
    # 2. Initialize Open3D Visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="ZED 3D Point Cloud", width=800, height=600)
    pcd = o3d.geometry.PointCloud()
    
    # Add a coordinate frame for orientation
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(axis)
    
    first_pcd = True
    


    # Use the length of the images list to drive the loop
    for i in range(len(reader.images)):
        
        img_time, img_data = reader.images[i]  # Get the timestamp and image from your module
        # --- Display RGB Image from your module ---
        # Your module handles the reshape and BGR conversion
        cv2.imshow('ZED RGB', img_data)

        print(reader.poses[i])
        
        # --- Display Depth Image from your module ---
        if i < len(reader.depths):
            depth_time, depth_img = reader.depths[i]

            #print(f"Depth Image {i}: {depth_img}")
            # Replace infinite values with a large finite value (e.g., 20 meters) for visualization
            depth_img = np.where(np.isfinite(depth_img), depth_img, 10.0)
            # Normalize and clip for visualization (0 to 5 meters)
            depth_viz = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
            cv2.imshow('ZED Depth', depth_viz)
            
            # Calculate min/max excluding infinite values
            # finite_depths = depth_img[np.isfinite(depth_img)]
            # if len(finite_depths) > 0:
            #     print(f"Depth Image {i}: Time={depth_time:.3f}s, Min={np.min(finite_depths):.2f}m, Max={np.max(finite_depths):.2f}m")
            # else:
            #     print(f"Depth Image {i}: Time={depth_time:.3f}s, All depths invalid")

         # --- Display point cloud Image from your module ---
        if( i < len(reader.point_clouds)):
            pc_time, xyz ,rgb= reader.point_clouds[i] # Already filtered for NaNs in your module

            # Update Open3D Geometry
            pcd.points = o3d.utility.Vector3dVector(xyz)
            
            # Update Open3D Geometry colors
            if rgb is not None:
                pcd.colors = o3d.utility.Vector3dVector(rgb)

            if first_pcd:
                vis.add_geometry(pcd)
                first_pcd = False
        
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        


        # Added a 33ms delay to simulate ~30 FPS playback speed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vis.destroy_window()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    display_zed_bag(BAG_PATH)
