import cv2
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
import threading
import queue

class ZEDBagReader:
    def __init__(self, bag_path_str: str):
        self.bag_path = Path(bag_path_str)

        # Mapping topics to their processing functions
        self.topic_map = {
            "/zed_head/zed_node/rgb/color/rect/image": self._handle_rgb,
            "/zed_head/zed_node/depth/depth_registered": self._handle_depth,
            "/zed_head/zed_node/point_cloud/cloud_registered": self._handle_pc,
            "/current_pose": self._handle_pose,
        }

    def read_chunks(self, chunk_size=100, prefetch_count=2):
        """
        Yields synchronized data chunks. 
        Uses a background thread to prefetch the next chunks so the main thread never waits.
        """
        # A thread-safe queue. maxsize=prefetch_count ensures we don't load the whole bag into memory.
        # If prefetch_count=2, it holds 1 chunk for you to use, and 1 chunk ready in the background.
        chunk_queue = queue.Queue(maxsize=prefetch_count)

        def producer():
            """This function runs in the background thread, reading the bag."""
            current_chunk = []
            latest_depth = None
            latest_pc = None
            latest_pose = None

            try:
                with AnyReader([self.bag_path]) as reader:
                    for connection, _, rawdata in reader.messages():
                        if connection.topic not in self.topic_map:
                            continue

                        msg = reader.deserialize(rawdata, connection.msgtype)
                        topic = connection.topic

                        if topic == "/zed_head/zed_node/depth/depth_registered":
                            latest_depth = self._handle_depth(msg)
                        
                        elif topic == "/zed_head/zed_node/point_cloud/cloud_registered":
                            latest_pc = self._handle_pc(msg)
                        
                        elif topic == "/current_pose":
                            latest_pose = self._handle_pose(msg)
                        
                        elif topic == "/zed_head/zed_node/rgb/color/rect/image":
                            # Use RGB as the trigger
                            img_time, img_data = self._handle_rgb(msg)
                            
                            frame = {
                                "time": img_time,
                                "image": img_data,
                                "depth": latest_depth,
                                "point_cloud": latest_pc,
                                "pose": latest_pose
                            }
                            
                            current_chunk.append(frame)

                            # When chunk is full, push it to the queue
                            if len(current_chunk) >= chunk_size:
                                # This blocks if the queue is full (i.e., main thread is slow)
                                chunk_queue.put(current_chunk) 
                                current_chunk = []

                # Put any leftover frames in the queue
                if len(current_chunk) > 0:
                    chunk_queue.put(current_chunk)
            finally:
                # Put a Sentinel value (None) to tell the main thread the bag is finished
                chunk_queue.put(None)

        # Start the background reader thread
        bg_thread = threading.Thread(target=producer, daemon=True)
        bg_thread.start()

        # The Main thread (Consumer) yields items from the queue
        while True:
            # This instantly returns if a chunk is prepared, otherwise waits slightly
            chunk = chunk_queue.get()
            if chunk is None:
                break # Reached the end of the bag
            
            yield chunk

    # --- Sensor Handlers (Keep exactly as before) ---
    def _handle_rgb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return self.extract_sensor_time(msg), img

    def _handle_depth(self, msg):
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
        return self.extract_sensor_time(msg), depth

    def _handle_pc(self, msg):
        fields = {f.name.lower(): f for f in msg.fields}
        if "x" not in fields or "y" not in fields or "z" not in fields: return None

        def get_dtype(ros_type): return np.float32 if ros_type == 7 else np.float64

        buf = memoryview(msg.data)
        stride = msg.point_step
        num_points = len(buf) // stride

        x = np.ndarray(shape=(num_points,), dtype=get_dtype(fields["x"].datatype), buffer=buf, offset=fields["x"].offset, strides=(stride,))
        y = np.ndarray(shape=(num_points,), dtype=get_dtype(fields["y"].datatype), buffer=buf, offset=fields["y"].offset, strides=(stride,))
        z = np.ndarray(shape=(num_points,), dtype=get_dtype(fields["z"].datatype), buffer=buf, offset=fields["z"].offset, strides=(stride,))
        xyz = np.stack([x, y, z], axis=-1).astype(np.float32)

        has_rgb = "rgb" in fields
        if has_rgb:
            rgb_float = np.ndarray(shape=(num_points,), dtype=np.float32, buffer=buf, offset=fields["rgb"].offset, strides=(stride,))
            rgb_uint32 = rgb_float.view(np.uint32)
            r = ((rgb_uint32 >> 16) & 255).astype(np.float64) / 255.0
            g = ((rgb_uint32 >> 8) & 255).astype(np.float64) / 255.0
            b = (rgb_uint32 & 255).astype(np.float64) / 255.0
            colors = np.stack([r, g, b], axis=-1)
        else:
            colors = None

        valid_xyz_mask = np.isfinite(xyz).all(axis=1)
        valid_xyz = xyz[valid_xyz_mask]
        valid_colors = colors[valid_xyz_mask] if has_rgb else None

        if len(valid_xyz) == 0: return None
        return self.extract_sensor_time(msg), valid_xyz, valid_colors

    def _handle_pose(self, msg):
        total_time = self.extract_sensor_time(msg)
        pos = msg.pose.pose.position
        position = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
        ori = msg.pose.pose.orientation
        quaternion = np.array([ori.x, ori.y, ori.z, ori.w], dtype=np.float64)
        return total_time, position, quaternion

    def extract_sensor_time(self, msg):
        return msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)