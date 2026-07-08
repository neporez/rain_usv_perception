"""
pose_cache.py

GNSS Pose(1Hz)를 저장해두고, 임의 시점의 global pose를
등속도 외삽(constant velocity extrapolation)으로 추정해서 제공한다.
"""
import math
import numpy as np


class PoseCache:
    def __init__(self):
        self._last_msg = None        # gnss_msgs/msg/Pose, 가장 최근 수신
        self._last_stamp_sec = None  # float, epoch seconds 기준

    def update(self, msg):
        self._last_msg = msg
        self._last_stamp_sec = msg.gnss_time  # UNIX TIME 필드 그대로 사용

    def has_fix(self) -> bool:
        return self._last_msg is not None

    def get_extrapolated_pose(self, query_time_sec: float) -> dict:
        if not self.has_fix():
            raise RuntimeError("No GNSS fix received yet.")

        dt = query_time_sec - self._last_stamp_sec

        cog_rad = math.radians(self._last_msg.cog)
        vx = self._last_msg.sog * math.sin(cog_rad)  # 동쪽(+x) 방향 속도
        vy = self._last_msg.sog * math.cos(cog_rad)  # 북쪽(+y) 방향 속도

        x_pred = self._last_msg.utmx + vx * dt
        y_pred = self._last_msg.utmy + vy * dt

        return {
            "x": x_pred,
            "y": y_pred,
            "z": self._last_msg.altitude,
            "yaw_deg": self._last_msg.yaw,
            "utmzone": self._last_msg.utmzone,
            "extrapolated_dt": dt,
        }

    def get_pose_matrix(self, query_time_sec: float) -> np.ndarray:
        pose = self.get_extrapolated_pose(query_time_sec)
        yaw_rad = math.radians(pose["yaw_deg"])

        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
        transform = np.eye(4)
        transform[0, 0], transform[0, 1] = cos_y, -sin_y
        transform[1, 0], transform[1, 1] = sin_y, cos_y
        transform[0, 3] = pose["x"]
        transform[1, 3] = pose["y"]
        transform[2, 3] = pose["z"]
        return transform