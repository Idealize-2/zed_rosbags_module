| Property | Data Type | Description |
| --- | --- | --- |
| .images | List[np.ndarray] | RGB images with shape (H, W, 3) and uint8 encoding. |
| .depths | List[np.ndarray] | Depth maps with shape (H, W) and float32 values representing meters. |
|.point_clouds|List[np.ndarray]|Filtered XYZ coordinates with shape (N, 3) in meters.|

N (Number of Points)

## How to use

```
from zed_bag_reader import ZEDBagReader

# Initialize the reader with the path to your bag folder
reader = ZEDBagReader('/home/walkie/rosbag/zed2i_2_bag')

# Access images like a standard Python list
first_image = reader.images[0]
total_depth_frames = len(reader.depths)

# Calculate distance to a specific pixel in the first frame
# (e.g., center pixel distance in meters)
h, w = reader.depths[0].shape
distance = reader.depths[0][h//2, w//2]
print(f"Distance to center: {distance:.2f}m")
```