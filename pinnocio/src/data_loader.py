import socket
import json
import time
import csv
import numpy as np
import threading
from typing import Dict, Optional

class HumanPoseSource:
    def __init__(self, mode: str, path: str = None, port: int = 8888):
        self.mode = mode
        self.path = path
        self.port = port
        self.current_pose: Optional[Dict[str, np.ndarray]] = None
        self.running = True
        
        self._lock = threading.Lock()

        if self.mode == 'udp':
            self._start_udp_listener()
        elif self.mode == 'csv':
            self._csv_generator = self._create_csv_generator()
        elif self.mode == 'bag':
            self._bag_generator = self._create_bag_generator()
        else:
            raise ValueError(f"Unknown mode: {mode}")

    def get_next_pose(self) -> Optional[Dict[str, np.ndarray]]:
        if self.mode == 'udp':
            with self._lock:
                return self.current_pose
        elif self.mode == 'csv':
            try:
                return next(self._csv_generator)
            except StopIteration:
                return None
        elif self.mode == 'bag':
            try:
                return next(self._bag_generator)
            except StopIteration:
                return None
        return None

    def _start_udp_listener(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(0.5)
        
        def listener():
            print(f"[DataLoader] Listening on UDP port {self.port}...")
            while self.running:
                try:
                    data, _ = self.sock.recvfrom(4096)
                    json_str = data.decode('utf-8')
                    pose = self._parse_json_skeleton(json_str)
                    if pose:
                        with self._lock:
                            self.current_pose = pose
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[DataLoader] UDP Error: {e}")
            self.sock.close()

        self.thread = threading.Thread(target=listener, daemon=True)
        self.thread.start()

    def _create_csv_generator(self):
        print(f"[DataLoader] Opening CSV: {self.path}")
        with open(self.path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pose = {}
                keys = row.keys()
                joints = set([k.replace('_X', '') for k in keys if k.endswith('_X')])
                
                for j in joints:
                    try:
                        x = float(row.get(f"{j}_X", 0))
                        y = float(row.get(f"{j}_Y", 0))
                        z = float(row.get(f"{j}_Z", 0))
                        pose[j] = np.array([z, -x, -y])
                    except ValueError:
                        continue
                
                if pose:
                    time.sleep(0.01) # 100fps cap
                    yield pose

    def _create_bag_generator(self):
        print(f"[DataLoader] Opening Rosbag: {self.path}")
        try:
            from rosbags.rosbag1 import Reader as Reader1
            from rosbags.rosbag2 import Reader as Reader2
            # Attempt to use serde if available, else standard
            # Simplified approach: Just text data?
            # If standard serde unavailable, maybe warn user
            from rosbags.serde import deserialize_cdr, ros1_to_cdr
        except ImportError as e:
            print(f"[DataLoader] Error importing rosbags: {e}")
            print("Please ensure 'rosbags' is installed and compatible.")
            return

        Reader = Reader2 if not self.path.endswith('.bag') else Reader1
        
        with Reader(self.path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                 if 'json' in connection.topic:
                     try:
                        msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                        json_str = msg.data
                        pose = self._parse_json_skeleton(json_str)
                        if pose:
                            time.sleep(0.01)
                            yield pose
                     except Exception as e:
                         pass

    def _parse_json_skeleton(self, json_str: str) -> Optional[Dict[str, np.ndarray]]:
        try:
            data = json.loads(json_str)
            skeletons = data.get('skeletons', [])
            if not skeletons:
                return None
            skel = skeletons[0]
            raw_joints = skel.get('joints', {})
            pose = {}
            for name, d in raw_joints.items():
                pose[name] = np.array([float(d['z']), -float(d['x']), -float(d['y'])])
            return pose
        except Exception:
            return None

    def close(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
