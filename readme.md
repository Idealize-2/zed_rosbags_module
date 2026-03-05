| Property | Data Type | Description |
| --- | --- | --- |
| .images |List[Tuple[float, np.ndarray]] | Hardware timestamp (seconds) and BGR images with shape (H, W, 3) in uint8 format. |
| .depths | List[Tuple[float, np.ndarray]] | Hardware timestamp (seconds) and Depth maps with shape (H, W) in float32 format (meters). |
|.point_clouds|List[Tuple[float, np.ndarray, np.ndarray]]]|Hardware timestamp (seconds), Filtered XYZ coordinates with shape (N, 3) in float32 format(meter), and RGB colors with shape (N, 3) in float64 format [0,1].|
|.poses|List[Tuple[float, np.ndarray, np.ndarray]]|Hardware timestamp (seconds), Position (x, y, z) in meters, and Orientation Quaternion (x, y, z, w).|

N (Number of Points)

## How to use

```
from zed_bag_reader import ZEDBagReader

# Initialize the reader with the path to your bag folder
reader = ZEDBagReader('/home/walkie/rosbag/zed2i_2_bag')

# Access data by unpacking the tuple (Timestamp, Data)
img_time, first_image = reader.images[0]
total_depth_frames = len(reader.depths)

# Calculate distance to a specific pixel in the first frame
# (e.g., center pixel distance in meters)
depth_time, first_depth = reader.depths[0]
h, w = first_depth.shape
distance = first_depth[h//2, w//2]

print(f"Data captured at: {depth_time:.4f} seconds")
print(f"Distance to center: {distance:.2f}m")

# Grab the very first pose
pose_time, position, quaternion = reader.poses[0]

print(f"X position: {position[0]:.2f}")
print(f"Heading (w): {quaternion[3]:.2f}")
```