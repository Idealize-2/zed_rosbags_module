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
        # Mapping topics to their processing functions
        self.topic_map = {
            '/zed/zed_node/rgb/color/rect/image': self._handle_rgb,
            '/zed/zed_node/depth/depth_registered': self._handle_depth,
            '/zed/zed_node/point_cloud/cloud_registered': self._handle_pc
        }
        
        self._load_data()

    def _load_data(self):
        """Processes the bag once and fills the lists."""
        with AnyReader([self.bag_path]) as reader:
            for connection, _, rawdata in reader.messages():
                if connection.topic in self.topic_map:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    self.topic_map[connection.topic](msg)
    

    def _handle_rgb(self, msg):
        # Reshape into (Height, Width, 3)
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # ZED is RGB, OpenCV wants BGR
        self._images.append(img)

    def _handle_depth(self, msg):
        # Use .copy() to avoid 'buffer is read-only' errors in some numpy versions
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
        self._depths.append(depth)

    def _handle_pc(self, msg):
        # 1. Parse Fields
        fields = {f.name.lower(): f for f in msg.fields}
        
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            return

        # 2. Determine Data Types
        def get_dtype(ros_type):
            return np.float32 if ros_type == 7 else np.float64

        dtype_x = get_dtype(fields['x'].datatype)
        dtype_y = get_dtype(fields['y'].datatype)
        dtype_z = get_dtype(fields['z'].datatype)

        # 3. Extract Data using Offsets and Strides
        buf = memoryview(msg.data)
        num_points = msg.width * msg.height
        stride = msg.point_step

        x = np.ndarray(shape=(num_points,), dtype=dtype_x, buffer=buf, 
                       offset=fields['x'].offset, strides=(stride,))
        y = np.ndarray(shape=(num_points,), dtype=dtype_y, buffer=buf, 
                       offset=fields['y'].offset, strides=(stride,))
        z = np.ndarray(shape=(num_points,), dtype=dtype_z, buffer=buf, 
                       offset=fields['z'].offset, strides=(stride,))

        # 4. Stack into (N, 3) array
        xyz = np.stack([x, y, z], axis=-1).astype(np.float32)
        
        # --- FIX: Filter out NaN AND Infinity ---
        # Simulation data often has 'inf' for sky/far objects, which breaks Open3D
        mask = np.isfinite(xyz).all(axis=1)
        valid_xyz = xyz[mask]

        self._point_clouds.append(valid_xyz)
    

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