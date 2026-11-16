#!/usr/bin/env python3
"""
replay_check.py -- 파이프라인 동작 확인: 일정 시간 동안 점군 / 탐지 / 추적 토픽을 세고 요약을 출력한다.

    source <repo>/env.sh
    ros2 launch lidar_perception_bringup perception.launch.py input:=packets use_sim_time:=true ouster_metadata_path:=<json> &
    ros2 bag play <bag> --clock &
    python3 <repo>/tools/replay_check.py --seconds 60

종료 코드 0 = 점군과 탐지 메시지를 모두 받았다. --expect-tracking 이면 추적 메시지도 있어야 0.
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock

from lidar_perception_interfaces.msg import Object3DArray, TrackedObject3DArray


class Checker(Node):
    def __init__(self, a):
        super().__init__("replay_check")
        be = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, a.points_topic, self.on_pc, be)
        self.create_subscription(Object3DArray, a.detection_topic, self.on_det, 10)
        self.create_subscription(TrackedObject3DArray, a.tracking_topic, self.on_trk, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, be)
        self.pc, self.det, self.trk, self.lag = [], [], [], []
        self.clock = None
        self.t_prev_det = None
        self.det_wall_dt = []

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_pc(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.pc.append((st, m.width * m.height, m.header.frame_id))
        if self.clock:
            self.lag.append(self.clock - st)

    def on_det(self, m):
        now = time.time()
        if self.t_prev_det is not None:
            self.det_wall_dt.append(now - self.t_prev_det)
        self.t_prev_det = now
        self.det.append(len(m.objects))

    def on_trk(self, m):
        self.trk.append(sorted(int(o.track_id) for o in m.objects))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--points-topic", default="/dev/lidar/points")
    ap.add_argument("--detection-topic", default="/detection/objects")
    ap.add_argument("--tracking-topic", default="/tracking/objects_viz")
    ap.add_argument("--expect-tracking", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = Checker(a)
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.2)
    st = np.array([p[0] for p in n.pc])
    print("=== replay_check (%.0f s) ===" % a.seconds)
    if n.pc:
        print("points   : %d msgs, %.1f Hz by stamp, %d pts/frame (median), frame_id %s, sim-clock minus stamp %.3f s"
              % (len(n.pc), 1.0 / np.median(np.diff(st)) if len(st) > 2 else 0.0, np.median([p[1] for p in n.pc]), n.pc[0][2], np.median(n.lag) if n.lag else float("nan")))
    else:
        print("points   : NONE")
    if n.det:
        print("detection: %d msgs, %.1f Hz (wall), objects/frame median %.0f max %d" % (len(n.det), 1.0 / np.median(n.det_wall_dt) if n.det_wall_dt else 0.0, np.median(n.det), max(n.det)))
    else:
        print("detection: NONE")
    if n.trk:
        ids = set(i for t in n.trk for i in t)
        print("tracking : %d msgs, tracks/frame median %.0f, distinct ids %d" % (len(n.trk), np.median([len(t) for t in n.trk]), len(ids)))
    else:
        print("tracking : NONE")
    ok = bool(n.pc) and bool(n.det) and (bool(n.trk) or not a.expect_tracking)
    print("RESULT:", "OK" if ok else "FAIL")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
