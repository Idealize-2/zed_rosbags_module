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
        # PointCloud2 to XYZ (ignoring color for speed)
        full_pc = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        xyz = full_pc[:, :3]
        self._point_clouds.append(xyz[~np.isnan(xyz).any(axis=1)])

    

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