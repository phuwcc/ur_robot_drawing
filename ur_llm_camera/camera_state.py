"""ROS camera subscriber and fresh, complete, stable observation barrier."""
import copy
import json
import math
import threading
import time

import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .perception import detect
from .scene import SceneError, occupancy


class CameraState:
    def __init__(self, node, config):
        cv2.setNumThreads(1)
        self.node = node
        self.config = config
        self.condition = threading.Condition()
        self.positions = {}
        self.received = 0.0
        self.stamp = -1.0
        self.stable = 0
        self.error = 'No camera image received'
        self.debug_pub = node.create_publisher(Image, '/camera/detections', 1)
        self.state_pub = node.create_publisher(String, '/perception/world_state', 10)
        self.sub = node.create_subscription(Image, config['camera']['image_topic'],
                                            self.on_image, qos_profile_sensor_data)

    def on_image(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        try:
            if message.encoding not in ('rgb8', 'bgr8') or message.step < message.width * 3:
                raise ValueError('Camera requires rgb8 or bgr8 image encoding')
            rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
            bgr = rows[:, :message.width*3].reshape(message.height, message.width, 3)
            if message.encoding == 'rgb8':
                bgr = bgr[:, :, ::-1].copy()
            positions, debug = detect(bgr, self.config)
            with self.condition:
                changed = set(positions) != set(self.positions) or any(
                    math.dist(p, self.positions[name]) > self.config['camera']['position_tolerance']
                    for name, p in positions.items() if name in self.positions)
                self.stable = 1 if changed or stamp <= self.stamp else self.stable + 1
                self.positions = positions
                self.received = time.monotonic()
                self.stamp = stamp
                self.error = ''
                self.condition.notify_all()
            msg = Image(height=debug.shape[0], width=debug.shape[1], encoding='bgr8',
                        step=debug.shape[1]*3, data=debug.tobytes())
            msg.header = message.header
            self.debug_pub.publish(msg)
            self.state_pub.publish(String(data=json.dumps({
                'stamp': stamp, 'frame_id': self.config['frame_id'], 'objects': positions,
                'zones': occupancy(positions, self.config),
                'complete': set(positions) == set(self.config['objects'])})))
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
                self.stable = 0
                self.condition.notify_all()

    def snapshot(self, after=None, required=None):
        camera = self.config['camera']
        deadline = time.monotonic() + camera['timeout']
        required = set(self.config['objects']) if required is None else set(required)
        with self.condition:
            while time.monotonic() < deadline:
                now = time.monotonic()
                sim_now = self.node.get_clock().now().nanoseconds / 1e9
                if (not self.error and required.issubset(self.positions) and
                    self.stable >= camera['stable_frames'] and
                    now-self.received <= camera['max_age'] and
                    0 <= sim_now-self.stamp <= camera['max_age'] and
                    (after is None or self.stamp > after)):
                    return copy.deepcopy(self.positions)
                self.condition.wait(timeout=0.1)
        missing = required.difference(self.positions)
        raise SceneError(f'PERCEPTION_UNAVAILABLE: {self.error}; missing={sorted(missing)}; '
                         'need fresh stable camera frames')
