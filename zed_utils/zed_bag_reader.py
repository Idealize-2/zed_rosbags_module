import cv2
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader

class ZEDBagReader:
    def __init__(self, bag_path_str: str):
        self.bag_path = Path(bag_path_str)
        self._images = []
        self._depths = []
        self._point_clouds = []
        self._poses = []  # <-- Added for pose

        # Mapping topics to their processing functions
        self.topic_map = {
            '/zed_head/zed_node/rgb/color/rect/image': self._handle_rgb,
            '/zed_head/zed_node/depth/depth_registered': self._handle_depth,
            '/zed_head/zed_node/point_cloud/cloud_registered': self._handle_pc,
            '/current_pose': self._handle_pose
        }
        
        self._load_data()

    def _load_data(self):
        """Processes the bag once and fills the lists."""
        with AnyReader([self.bag_path]) as reader:
            for connection, _, rawdata in reader.messages():
                if connection.topic in self.topic_map:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    self.topic_map[connection.topic](msg)

        # Sort lists chronologically based on the first element of the tuple (the timestamp)
        self._images.sort(key=lambda x: x[0])
        self._depths.sort(key=lambda x: x[0])
        self._point_clouds.sort(key=lambda x: x[0])
        self._poses.sort(key=lambda x: x[0])

    def _handle_rgb(self, msg):
        # Reshape into (Height, Width, 3)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # ZED is RGB, OpenCV wants BGR
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        total_times = self.extract_sensor_time(msg)  # You can store or use this timestamp as needed
        self._images.append((total_times, img))

    def _handle_depth(self, msg):
        # Use .copy() to avoid 'buffer is read-only' errors in some numpy versions
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
        total_times = self.extract_sensor_time(msg)  # You can store or use this timestamp as needed
        self._depths.append((total_times, depth))

    def _handle_pc(self, msg):
        # 1. Parse Fields
        fields = {f.name.lower(): f for f in msg.fields}
        
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            return

        def get_dtype(ros_type):
            return np.float32 if ros_type == 7 else np.float64

        buf = memoryview(msg.data)
        stride = msg.point_step
        num_points = len(buf) // stride 

        # 2. Extract XYZ
        x = np.ndarray(shape=(num_points,), dtype=get_dtype(fields['x'].datatype), buffer=buf, offset=fields['x'].offset, strides=(stride,))
        y = np.ndarray(shape=(num_points,), dtype=get_dtype(fields['y'].datatype), buffer=buf, offset=fields['y'].offset, strides=(stride,))
        z = np.ndarray(shape=(num_points,), dtype=get_dtype(fields['z'].datatype), buffer=buf, offset=fields['z'].offset, strides=(stride,))
        xyz = np.stack([x, y, z], axis=-1).astype(np.float32)

        # 3. Extract True Colors Using Standard ROS Float32 Bitwise Unpacking
        has_rgb = 'rgb' in fields
        if has_rgb:
            # Read the rgb field as a float32 array
            rgb_float = np.ndarray(shape=(num_points,), dtype=np.float32, buffer=buf, offset=fields['rgb'].offset, strides=(stride,))
            
            # View the float32 bits safely as uint32 integers
            rgb_uint32 = rgb_float.view(np.uint32)
            
            # ROS packs RGB into uint32: 0x00RRGGBB (or BGRA depending on camera)
            # Standard bitwise shift to extract the channels safely
            r = ((rgb_uint32 >> 16) & 255).astype(np.float64) / 255.0
            g = ((rgb_uint32 >> 8) & 255).astype(np.float64) / 255.0
            b = (rgb_uint32 & 255).astype(np.float64) / 255.0
            
            # Note: If colors look swapped (e.g. Red looks Blue), simply swap 'r' and 'b' in this stack:
            colors = np.stack([r, g, b], axis=-1)
        else:
            colors = None

        # 4. Remove NaN/Inf points
        valid_xyz_mask = np.isfinite(xyz).all(axis=1)
        
        valid_xyz = xyz[valid_xyz_mask]
        valid_colors = colors[valid_xyz_mask] if has_rgb else None

        if len(valid_xyz) == 0:
            return

        total_times = self.extract_sensor_time(msg)
        self._point_clouds.append((total_times, valid_xyz, valid_colors))
    
    def _handle_pose(self, msg):
        total_time = self.extract_sensor_time(msg)
        
        # Extract position
        pos = msg.pose.pose.position
        position = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        
        # Extract orientation (quaternion)
        ori = msg.pose.pose.orientation
        quaternion = np.array([ori.x, ori.y, ori.z, ori.w], dtype=np.float64)
        
        # We append a tuple of: (timestamp, position_array, quaternion_array)
        self._poses.append((total_time, position, quaternion))

    def extract_sensor_time(self, msg):
        # msg.header.stamp contains 'sec' and 'nanosec'
        seconds = msg.header.stamp.sec
        nanoseconds = msg.header.stamp.nanosec
        
        # Combine them into a single float (total seconds)
        total_time = seconds + (nanoseconds * 1e-9)
        return total_time
    

    # Properties make the class act like a data structure
    @property
    def images(self):
        return self._images

    @property
    def depths(self):
        return self._depths

    @property
    def point_clouds(self):
        return self._point_clouds
    
    @property
    def poses(self):
        return self._poses