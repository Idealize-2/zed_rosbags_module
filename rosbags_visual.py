import cv2
import open3d as o3d
from zed_utils import ZEDBagReader  # Importing your custom module

BAG_PATH = '/home/walkie/rosbag/zed2i_2_bag'

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

    # Use the length of the images list to drive the loop
    for i in range(len(reader.images)):
        
        # --- Display RGB Image from your module ---
        # Your module handles the reshape and BGR conversion
        cv2.imshow('ZED RGB', reader.images[i])

        # --- Display Depth Image from your module ---
        if i < len(reader.depths):
            depth_img = reader.depths[i]
            # Normalize and clip for visualization (0 to 5 meters)
            depth_viz = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX).astype('uint8')
            cv2.imshow('ZED Depth', depth_viz)

        # --- Update Point Cloud from your module ---
        if i < len(reader.point_clouds):
            xyz = reader.point_clouds[i] # Already filtered for NaNs in your module

            # Update Open3D Geometry
            pcd.points = o3d.utility.Vector3dVector(xyz)
            
            if first_pcd:
                vis.add_geometry(pcd)
                first_pcd = False
            
            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

        # Added a 33ms delay to simulate ~30 FPS playback speed
        if cv2.waitKey(33) & 0xFF == ord('q'):
            break

    vis.destroy_window()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    display_zed_bag(BAG_PATH)