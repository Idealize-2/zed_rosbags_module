import cv2
import open3d as o3d
from zed_utils import ZEDBagReader  # Importing your custom module

#BAG_PATH = '/home/walkie/robocup2026/rosbag/13022026-SImulation'


BAG_PATH = '../sim_real_rosbag.tar/sim_real_rosbag/real_14022026'

def display_zed_bag(bag_path_str):
    # 1. Initialize your ZEDBagReader module
    # This will load all images, depths, and point clouds into memory
    reader = ZEDBagReader(bag_path_str)
    
    # 2. Initialize Open3D Visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="ZED 3D Point Cloud", width=800, height=600)
    pcd = o3d.geometry.PointCloud()
    
    # Add a coordinate frame for orientation
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(axis)
    
    first_pcd = True

    j = 0  # Point cloud index pointer
    max_j = len(reader.point_clouds) - 1

    # Use the length of the images list to drive the loop
    for i in range(len(reader.images)):
        
        img_time, img_data = reader.images[i]  # Get the timestamp and image from your module
        # --- Display RGB Image from your module ---
        # Your module handles the reshape and BGR conversion
        cv2.imshow('ZED RGB', img_data)

        # --- Display Depth Image from your module ---
        if i < len(reader.depths):
            depth_time, depth_img = reader.depths[i]
            # Normalize and clip for visualization (0 to 5 meters)
            depth_viz = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
            cv2.imshow('ZED Depth', depth_viz)

        # --- Update Point Cloud from your module ---
        while j < max_j:
            current_diff = abs(img_time - reader.point_clouds[j][0])
            next_diff = abs(img_time - reader.point_clouds[j+1][0])
            
            # If the NEXT point cloud is a better match in time, move forward
            if next_diff < current_diff:
                j += 1
            else:
                # We found the closest match, stop advancing
                break
        pc_time, xyz = reader.point_clouds[j] # Already filtered for NaNs in your module

        # Update Open3D Geometry
        pcd.points = o3d.utility.Vector3dVector(xyz)
        
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