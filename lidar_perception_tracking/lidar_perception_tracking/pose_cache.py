## @file
#  @brief GNSS/INS pose 를 시간순 버퍼에 저장해 LiDAR 프레임 시각의 pose 를 보간으로 제공하는 캐시.

"""!
@brief GNSS/INS pose 를 시간순 버퍼에 저장해 LiDAR 프레임 시각의 pose 를 보간으로 제공하는 캐시.

GNSS / INS pose (gnss_msgs/msg/Pose) 를 시간순 버퍼에 저장해 두고, 라이다 프레임 시각의 자선 pose 를
**버퍼 안의 두 샘플 사이 보간**으로 돌려준다. 속도 외삽은 하지 않는다.

배포 시스템의 pose 소스 (2026-11 bag 기준):
    /dev/gnss/hemisphere/pose   10 Hz  (= /system/gnss/pose, 같은 내용)   <- 기본값
    /dev/spatial_fog_dual/pose  100 Hz (INS)
    /digital_bridge/gnss/pose   1 Hz   (예전 기본값; 외삽이 필요했던 이유)
10 Hz 이상에서는 라이다 시각(10 Hz)이 항상 두 pose 샘플 사이에 들어오므로 보간으로 충분하다.

시각 기준 (time_source):
    'auto'      (기본) pose 마다 header.stamp 와 수신 시각(노드 시계)을 둘 다 기억해 두고, 질의(라이다 프레임 시각)에
                더 가까운 쪽을 기준으로 보간한다. 그래서 bag 을 --clock 없이 재생하거나 use_sim_time 을 빼도
                (라이다 스탬프 = 현재 시각, pose header = 녹화 당시 시각) 추적이 동작한다.
    'header'    pose 메시지의 header.stamp (실센서: 호스트 시계, 라이다 PointCloud2 와 같은 시계).
    'receive'   pose 를 받은 노드 시계 시각.
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
    """!
    @brief gnss_msgs/Pose 를 시간순으로 쌓아 두고 임의 시각(LiDAR 프레임)의 자선 pose 를 보간해 돌려주는 버퍼.

    LiDAR 탐지는 10Hz, pose 는 10Hz(hemisphere) 또는 100Hz(Spatial FOG)로 들어오므로 LiDAR 프레임 시각은
    거의 항상 두 pose 샘플 사이에 놓인다. 위치는 선형, yaw 는 최단각으로 보간하고, 버퍼 끝 밖이면
    max_age_sec 까지만 가장 가까운 샘플을 그대로 쓴다(속도 외삽 없음). tracking_node 가 매 탐지 프레임마다
    get_pose_matrix() 를 호출해 로컬→글로벌(UTM, ENU) 변환 행렬을 얻는다.
    """

    def __init__(self, max_age_sec: float = 0.5, buffer_sec: float = 5.0, time_source: str = "auto",
                 heading_convention: str = "ned", time_offset_sec: float = 0.0):
        """!
        @brief 빈 버퍼로 초기화.
        @param max_age_sec 버퍼 밖 시각을 허용하는 최대 거리 [s]; 넘으면 get_pose() 가 None.
        @param buffer_sec 버퍼에 유지하는 시간 길이 [s].
        @param time_source 'auto'(질의 시각에 가까운 쪽 자동 선택) | 'header' | 'receive' | 'gnss_time'.
        @param heading_convention 'ned'(Pose.yaw = 진북 기준 시계방향 deg → ENU yaw 로 변환) | 'enu'(그대로).
        @param time_offset_sec 질의 시각에 더하는 보정값 [s].
        """
        if time_source not in ("auto", "header", "receive", "gnss_time"):
            raise ValueError(f"time_source must be 'auto', 'header', 'receive' or 'gnss_time', got {time_source!r}")
        if heading_convention not in ("ned", "enu"):
            raise ValueError(f"heading_convention must be 'ned' or 'enu', got {heading_convention!r}")
        self.max_age_sec = float(max_age_sec)        # 버퍼 밖 시각을 허용하는 최대 거리 (가장 가까운 샘플을 그대로 사용)
        self.buffer_sec = float(buffer_sec)          # 버퍼에 유지하는 시간 길이
        self.time_source = time_source
        self.heading_convention = heading_convention
        self.time_offset_sec = float(time_offset_sec)   # 질의 시각에 더하는 보정값 (라이다 / pose 시계 차이가 있을 때)
        ## pose 시각 [s] (header 또는 gnss_time 기준), 단조 증가
        self._t = deque()
        ## pose 수신 시각 [s] (노드 시계), _t 와 같은 순서 ('auto' / 'receive' 에서 사용)
        self._rx = deque()
        ## (x, y, z, yaw_rad, utmzone) — 시각과 같은 순서
        self._p = deque()
        ## 마지막 get_pose() 가 쓴 시각 기준 ('header' | 'receive' | 'gnss_time'), 로그용
        self.last_base = None
        ## 수신한 pose 수
        self.n_received = 0

    # ------------------------------------------------------------------ input
    def update(self, msg, receive_time_sec=None):
        """!
        @brief 새 pose 를 버퍼 끝에 추가 (역순/중복 스탬프는 버림), buffer_sec 보다 오래된 샘플은 제거.
        @param msg gnss_msgs/msg/Pose 메시지.
        @param receive_time_sec 수신 시각 [s] (노드 시계). None 이면 header.stamp 로 대신한다.
        """
        hdr = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = float(msg.gnss_time) if self.time_source == "gnss_time" else hdr
        rx = hdr if receive_time_sec is None else float(receive_time_sec)
        if self._t and t <= self._t[-1]:
            # 역순 / 중복 스탬프는 버린다 (보간 구간이 꼬이지 않도록)
            return
        yaw_deg = msg.yaw if self.heading_convention == "enu" else (90.0 - msg.yaw)
        self._t.append(t)
        self._rx.append(rx)
        self._p.append((float(msg.utmx), float(msg.utmy), float(msg.altitude), math.radians(yaw_deg), msg.utmzone))
        self.n_received += 1
        while self._t and t - self._t[0] > self.buffer_sec:
            self._t.popleft()
            self._rx.popleft()
            self._p.popleft()

    def has_fix(self) -> bool:
        """!
        @brief pose 를 한 번이라도 수신했는지 여부.
        @return 버퍼가 비어 있지 않으면 True.
        """
        return len(self._t) > 0

    def latest_time(self):
        """!
        @brief 가장 최근 pose 의 시각.
        @return 시각 [s], 버퍼가 비었으면 None.
        """
        return self._t[-1] if self._t else None

    # ------------------------------------------------------------------ query
    def get_pose(self, query_time_sec: float):
        """!
        @brief 라이다 프레임 시각의 pose 를 보간해 돌려준다.

        질의 시각(+ time_offset_sec)이 두 샘플 사이면 위치 선형 / yaw 최단각 보간('interp'), 버퍼 끝 밖이면
        가장 가까운 끝 샘플을 그대로 사용('hold', |dt| <= max_age_sec), 그보다 멀면 None (외삽하지 않음).

        @param query_time_sec pose 를 구할 시각 [s] (LiDAR 프레임 header.stamp).
        @return dict(x, y, z, yaw_rad, utmzone, dt, mode) 또는 None.
        """
        if not self._t:
            return None
        q = query_time_sec + self.time_offset_sec
        # 시각 기준 선택: 'auto' 는 최신 샘플이 질의 시각에 더 가까운 쪽 (header/gnss_time vs 수신 시각)
        if self.time_source == "receive":
            ts, base = self._rx, "receive"
        elif self.time_source == "auto" and abs(self._rx[-1] - q) < abs(self._t[-1] - q):
            ts, base = self._rx, "receive"
        else:
            ts, base = self._t, self.time_source if self.time_source != "auto" else "header"
        self.last_base = base
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
        """!
        @brief get_pose() 결과를 4x4 동차변환행렬(센서 로컬 → 글로벌 UTM/ENU)로 변환.
        @param query_time_sec pose 를 구할 시각 [s].
        @return numpy (4,4) 행렬, 사용할 pose 가 없으면 None.
        """
        pose = self.get_pose(query_time_sec)
        if pose is None:
            return None
        c, s = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
        T = np.eye(4)
        T[0, 0], T[0, 1] = c, -s
        T[1, 0], T[1, 1] = s, c
        T[0, 3], T[1, 3], T[2, 3] = pose["x"], pose["y"], pose["z"]
        return T
