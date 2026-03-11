| Property | Data Type | Description |
| --- | --- | --- |
| .images |List[Tuple[float, np.ndarray]] | Hardware timestamp (seconds) and BGR images with shape (H, W, 3) in uint8 format. |
| .depths | List[Tuple[float, np.ndarray]] | Hardware timestamp (seconds) and Depth maps with shape (H, W) in float32 format (meters). |
|.point_clouds|List[Tuple[float, np.ndarray, np.ndarray]]]|Hardware timestamp (seconds), Filtered XYZ coordinates with shape (N, 3) in float32 format(meter), and RGB colors with shape (N, 3) in float64 format [0,1].|
|.poses|List[Tuple[float, np.ndarray, np.ndarray]]|Hardware timestamp (seconds), Position (x, y, z) in meters, and Orientation Quaternion (x, y, z, w).|

N (Number of Points)

## How to use

```python
from zed_utils import ZEDBagReader

# Initialize the reader
reader = ZEDBagReader('/home/walkie/rosbag/zed2i_2_bag')

# Process the bag in chunks of 50 frames to avoid memory overload
for chunk in reader.read_chunks(chunk_size=50):
    for frame in chunk:
        print(f"Time: {frame['time']}")
        
        # Access RGB
        img = frame['image']
        
        # Access Depth (if available for this frame)
        if frame['depth']:
            depth_time, depth_map = frame['depth']
            
        # Access Point Cloud (if available)
        if frame['point_cloud']:
            pc_time, xyz, rgb = frame['point_cloud']
            
        # Access Pose (if available)
        if frame['pose']:
            pose_time, position, quaternion = frame['pose']
```