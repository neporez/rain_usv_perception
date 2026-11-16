"""
pose_cache.py

GNSS / INS pose (gnss_msgs/msg/Pose) 를 시간순 버퍼에 저장해 두고, 라이다 프레임 시각의 자선 pose 를
**버퍼 안의 두 샘플 사이 보간**으로 돌려준다. 속도 외삽은 하지 않는다.

배포 시스템의 pose 소스 (2026-11 bag 기준):
    /dev/gnss/hemisphere/pose   10 Hz  (= /system/gnss/pose, 같은 내용)   <- 기본값
    /dev/spatial_fog_dual/pose  100 Hz (INS)
    /digital_bridge/gnss/pose   1 Hz   (예전 기본값; 외삽이 필요했던 이유)
10 Hz 이상에서는 라이다 시각(10 Hz)이 항상 두 pose 샘플 사이에 들어오므로 보간으로 충분하다.

시각 기준 (time_source):
    'header'    pose 메시지의 header.stamp = 수신 시각 (호스트 시계). 라이다 PointCloud2 의 header.stamp 도
                드라이버가 같은 호스트 시계로 찍으므로 (스캔 시작 시각) 기본값으로 이것을 쓴다.
    'gnss_time' NMEA 의 UNIX time (fix 시각). hemisphere 는 header 보다 ~0.13 s 앞선다. 호스트 시계가 GPS 와
                동기되어 있을 때만 의미가 있다.

방위 (heading_convention):
    'ned'  gnss_msgs/Pose.yaw = 진북 0 deg, 시계 방향 증가 (메시지 정의) -> ENU x 축(동) 기준 반시계 yaw 로 변환
           (yaw_enu = 90 - heading). UTM (x 동, y 북) 위치와 같은 좌표계가 된다.
    'enu'  yaw 를 이미 ENU 반시계 각(deg)으로 보고 그대로 사용 (예전 코드와 같은 동작).
"""
import bisect
import math
from collections import deque

import numpy as np


class PoseCache:
    def __init__(self, max_age_sec: float = 0.5, buffer_sec: float = 5.0, time_source: str = "header",
                 heading_convention: str = "ned", time_offset_sec: float = 0.0):
        if time_source not in ("header", "gnss_time"):
            raise ValueError(f"time_source must be 'header' or 'gnss_time', got {time_source!r}")
        if heading_convention not in ("ned", "enu"):
            raise ValueError(f"heading_convention must be 'ned' or 'enu', got {heading_convention!r}")
        self.max_age_sec = float(max_age_sec)        # 버퍼 밖 시각을 허용하는 최대 거리 (가장 가까운 샘플을 그대로 사용)
        self.buffer_sec = float(buffer_sec)          # 버퍼에 유지하는 시간 길이
        self.time_source = time_source
        self.heading_convention = heading_convention
        self.time_offset_sec = float(time_offset_sec)   # 질의 시각에 더하는 보정값 (라이다 / pose 시계 차이가 있을 때)
        self._t = deque()        # float seconds, 단조 증가
        self._p = deque()        # (x, y, z, yaw_rad, utmzone)
        self.n_received = 0

    # ------------------------------------------------------------------ input
    def update(self, msg):
        if self.time_source == "header":
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            t = float(msg.gnss_time)
        if self._t and t <= self._t[-1]:
            # 역순 / 중복 스탬프는 버린다 (보간 구간이 꼬이지 않도록)
            return
        yaw_deg = msg.yaw if self.heading_convention == "enu" else (90.0 - msg.yaw)
        self._t.append(t)
        self._p.append((float(msg.utmx), float(msg.utmy), float(msg.altitude), math.radians(yaw_deg), msg.utmzone))
        self.n_received += 1
        while self._t and t - self._t[0] > self.buffer_sec:
            self._t.popleft()
            self._p.popleft()

    def has_fix(self) -> bool:
        return len(self._t) > 0

    def latest_time(self):
        return self._t[-1] if self._t else None

    # ------------------------------------------------------------------ query
    def get_pose(self, query_time_sec: float):
        """라이다 프레임 시각의 pose. dict(x, y, z, yaw_rad, utmzone, dt, mode) 또는 None (사용할 pose 없음).
        mode: 'interp' 두 샘플 사이 보간 | 'hold' 버퍼 끝 샘플 그대로 (|dt| <= max_age_sec)."""
        if not self._t:
            return None
        q = query_time_sec + self.time_offset_sec
        ts = self._t
        i = bisect.bisect_left(ts, q)
        if 0 < i < len(ts):
            t0, t1 = ts[i - 1], ts[i]
            w = (q - t0) / (t1 - t0) if t1 > t0 else 0.0
            p0, p1 = self._p[i - 1], self._p[i]
            x = p0[0] + w * (p1[0] - p0[0])
            y = p0[1] + w * (p1[1] - p0[1])
            z = p0[2] + w * (p1[2] - p0[2])
            yaw = math.atan2((1 - w) * math.sin(p0[3]) + w * math.sin(p1[3]),
                             (1 - w) * math.cos(p0[3]) + w * math.cos(p1[3]))
            return {"x": x, "y": y, "z": z, "yaw_rad": yaw, "utmzone": p1[4], "dt": 0.0, "mode": "interp"}
        # 버퍼 밖: 가장 가까운 끝 샘플을 그대로 쓴다 (외삽 없음), 너무 멀면 None
        j = 0 if i == 0 else len(ts) - 1
        dt = q - ts[j]
        if abs(dt) > self.max_age_sec:
            return None
        p = self._p[j]
        return {"x": p[0], "y": p[1], "z": p[2], "yaw_rad": p[3], "utmzone": p[4], "dt": dt, "mode": "hold"}

    def get_pose_matrix(self, query_time_sec: float):
        """4x4 sensor -> global(UTM, ENU) 변환. pose 가 없으면 None."""
        pose = self.get_pose(query_time_sec)
        if pose is None:
            return None
        c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
        T = np.eye(4)
        T[0, 0], T[0, 1] = c, -s
        T[1, 0], T[1, 1] = s, c
        T[0, 3], T[1, 3], T[2, 3] = pose["x"], pose["y"], pose["z"]
        return T
