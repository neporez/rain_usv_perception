## @file
#  @brief ROS 메시지 ↔ Tracker3D 라이브러리 포맷을 잇는 어댑터.

"""! @brief ROS 메시지 ↔ Tracker3D 라이브러리 포맷을 잇는 TrackerAdapter. """
import logging
import os

import numpy as np

from lidar_perception_tracking.tracker.tracker import Tracker3D
from lidar_perception_tracking.tracker.config import cfg, cfg_from_yaml_file
from lidar_perception_tracking.tracker.box_op import register_bbs, get_registration_angle

## 원본 탐지와 매칭 실패 등 이 모듈의 경고/디버그 로그 출력용
logger = logging.getLogger(__name__)


class TrackerAdapter:
    """!
    @brief ROS 메시지 필드(Object3DArray 유래 배열) ↔ Tracker3D 라이브러리 포맷을 잇는 어댑터.

    Tracker3D는 (N,7) OpenPCDet 박스 배열과 글로벌 pose 행렬만 다루는 범용
    3D MOT 라이브러리라서, ROS 쪽에서 필요한 class_id/class_name/score 같은
    부가 필드와 로컬 좌표계 변환, 속도 추정은 이 클래스가 직접 계산해서 채워준다.

    주요 구성:
    - __init__(): tracker 설정(yaml) 로드, Tracker3D 인스턴스 생성.
    - update(): 한 프레임 추적 수행, ROS에서 쓰기 좋은 dict 리스트로 반환.
    """

    def __init__(self, cfg_path: str):
        """!
        @brief tracker 설정 파일을 로드하고 Tracker3D를 생성.
        @param cfg_path tracker 설정 yaml 경로 (이 파일이 있는 패키지 디렉토리 기준 상대경로).
        """
        _PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
        cfg_path = os.path.join(_PACKAGE_DIR, cfg_path)
        ## tracker/config/*.yaml에서 로드한 KF 파라미터 (state_func_covariance 등)
        self.config = cfg_from_yaml_file(cfg_path, cfg)
        ## OpenPCDet 박스 포맷 기준으로 동작하는 3D 칼만필터 트래커
        self.tracker = Tracker3D(
            box_type="OpenPCDet",
            tracking_features=False,
            config=self.config,
        )
        ## Tracker3D.tracking()에 넘기는 단조증가 프레임 번호 (실제 timestamp가 아니라 순번)
        self._frame_idx = 0
        ## {track_id: (gx, gy, gz, timestamp_sec)} — 직전 프레임 글로벌 위치, 속도 계산용
        self._prev_states = {}

    def update(self, boxes, scores, class_ids, class_names, pose, timestamp_sec):
        """!
        @brief 한 프레임의 탐지 결과를 추적기에 넣고, ID/로컬좌표/속도가 채워진 결과를 반환.

        1. **글로벌 변환 재현**: register_bbs()로 tracker 내부와 동일하게 박스를 글로벌 좌표로 변환해둔다 (이후 tracker 출력과 원본 탐지를 매칭하기 위함).
        2. **추적**: Tracker3D.tracking()으로 ID 할당 및 칼만필터 업데이트를 수행한다.
        3. **원본 매칭**: tracker가 반환한 글로벌 박스를 1번의 global_boxes_all과 좌표값이 거의 같은(atol=1e-6) 것으로 역매칭해서, 원본 class_id/class_name/score를 되찾는다.
        4. **로컬 변환 및 속도 추정**: pose의 역행렬로 로컬 좌표를 복원하고, 직전 프레임(_prev_states)과의 위치차/시간차로 속도를 추정한다 (첫 프레임은 속도 0).

        @param boxes (N,7) 배열 [x,y,z,dx,dy,dz,yaw], 센서 로컬 좌표계 기준.
        @param scores (N,) 탐지 신뢰도.
        @param class_ids (N,) 클래스 인덱스.
        @param class_names (N,) 클래스 이름.
        @param pose 4x4 로컬→글로벌 변환 행렬 (PoseCache.get_pose_matrix() 결과).
        @param timestamp_sec 현재 프레임 시각 (epoch seconds), 속도 계산의 dt에 사용.
        @return track_id/local_center/local_yaw/size/velocity/global_position/global_yaw/class_id/class_name/score를 담은 dict 리스트.
        """
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