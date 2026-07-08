"""
tracker_adapter.py
"""
import logging
import os

import numpy as np

from lidar_perception_tracking.tracker.tracker import Tracker3D
from lidar_perception_tracking.tracker.config import cfg, cfg_from_yaml_file
from lidar_perception_tracking.tracker.box_op import register_bbs, get_registration_angle

logger = logging.getLogger(__name__)


class TrackerAdapter:
    def __init__(self, cfg_path: str):
        _PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
        cfg_path = os.path.join(_PACKAGE_DIR, cfg_path)
        self.config = cfg_from_yaml_file(cfg_path, cfg)
        self.tracker = Tracker3D(
            box_type="OpenPCDet",
            tracking_features=False,
            config=self.config,
        )
        self._frame_idx = 0
        self._prev_states = {}

    def update(self, boxes, scores, class_ids, class_names, pose, timestamp_sec):
        # tracker에 넘길 입력은 별도 복사본으로 (tracker가 내부적으로 또 복사하긴 하지만, 방어적으로)
        boxes_for_tracker = boxes.copy()

        # 우리가 직접 register_bbs를 재현해서, tracker 내부와 동일한 "글로벌 변환 후" 값을 확보
        global_boxes_all = register_bbs(boxes.copy(), pose)

        tracked_bbs, ids = self.tracker.tracking(
            boxes_for_tracker,
            features=None,
            scores=scores,
            pose=pose,
            timestamp=self._frame_idx,
        )
        self._frame_idx += 1

        pose_inv = np.linalg.inv(pose)
        ang = get_registration_angle(pose)

        results = []
        current_states = {}

        for bb, track_id in zip(tracked_bbs, ids):
            track_id = int(track_id)

            match_idx = np.where(np.all(np.isclose(global_boxes_all, bb, atol=1e-6), axis=1))[0]
            if len(match_idx) == 0:
                logger.warning(
                    "TrackerAdapter: no matching source detection for track_id=%d, skipping.",
                    track_id,
                )
                continue
            orig_idx = match_idx[0]

            gx, gy, gz, dx, dy, dz, g_yaw = bb
            global_xyz1 = np.array([gx, gy, gz, 1.0])
            lx, ly, lz = (pose_inv @ global_xyz1)[:3]
            l_yaw = g_yaw - ang

            if track_id in self._prev_states:
                px, py, pz, pt = self._prev_states[track_id]
                dt = timestamp_sec - pt
                vx, vy, vz = ((gx - px) / dt, (gy - py) / dt, (gz - pz) / dt) if dt > 0 else (0.0, 0.0, 0.0)
            else:
                vx, vy, vz = 0.0, 0.0, 0.0

            current_states[track_id] = (gx, gy, gz, timestamp_sec)

            results.append({
                "track_id": track_id,
                "local_center": (float(lx), float(ly), float(lz)),
                "local_yaw": float(l_yaw),
                "size": (float(dx), float(dy), float(dz)),
                "velocity": (vx, vy, vz),
                "global_position": (float(gx), float(gy), float(gz)),
                "global_yaw": float(g_yaw),
                "class_id": int(class_ids[orig_idx]),
                "class_name": class_names[orig_idx],
                "score": float(scores[orig_idx]),
            })

        self._prev_states = current_states
        return results