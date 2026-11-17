## @file
#  @brief OpenPCDet(VoxelNeXt) 모델 로드 및 추론 래퍼.

"""! @brief OpenPCDet(VoxelNeXt) 모델 로드 및 추론 래퍼. """
import numpy as np
import torch
import yaml

from pcdet.config import cfg, merge_new_config
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils

import os


class _InferenceDataset(DatasetTemplate):
    """!
    @brief 파일 스캔 없이 prepare_data()/collate_batch()만 재사용하기 위한 경량 DatasetTemplate 서브클래스.

    OpenPCDet의 DatasetTemplate은 원래 디스크에서 데이터셋을 스캔하는 학습/평가용
    클래스지만, 실시간 추론에서는 ROS 토픽으로 들어온 포인트클라우드 1개만 그때그때
    전처리하면 되므로 파일 목록이 필요 없다. 그래서 `__len__`은 0, `__getitem__`은
    미구현으로 두고, 전처리 파이프라인(prepare_data/collate_batch)만 상속받아 쓴다.
    배포용 설정(kriso_voxelnext_deploy.yaml)에는 데이터 경로·infos·증강이 없으므로 그대로 동작한다.
    """

    def __init__(self, dataset_cfg, class_names, training=False):
        """!
        @brief DatasetTemplate 초기화 (파일 스캔 없이 전처리 설정만 구성).

        @param dataset_cfg 데이터 전처리 설정 (voxel size, point cloud range 등).
        @param class_names 클래스 이름 목록.
        @param training 학습 모드 여부 (추론 전용이므로 항상 False로 사용됨).
        """
        super().__init__(
            dataset_cfg=dataset_cfg,
            class_names=class_names,
            training=training,
            root_path=None,
            logger=None,
        )

    def __len__(self):
        """! @brief 파일 스캔을 하지 않으므로 항상 0. """
        return 0

    def __getitem__(self, index):
        """!
        @brief 사용되지 않음 (실시간 추론에서는 prepare_data()를 직접 호출).
        @param index 미사용.
        """
        raise NotImplementedError


class ModelInferencer:
    """!
    @brief VoxelNeXt(OpenPCDet) 체크포인트를 로드해 포인트클라우드 → 9자유도 3D bbox 추론을 수행하는 래퍼.

    설정 파일(cfg_path)과 체크포인트(ckpt_path)를 읽어 OpenPCDet 모델을 GPU에
    올려두고, predict()가 호출될 때마다 전처리 → 추론 → score_thresh 필터링 →
    Object3D 필드 스키마 변환까지 한 번에 처리한다. 모델이 학습한 점 특징 수
    (KRISO VoxelNeXt: x, y, z 3개)만 입력에서 잘라 쓰고, 상자는 7열(yaw) / 9열(yaw, roll, pitch)
    모두 처리한다.
    """

    def __init__(self, cfg_path: str, ckpt_path: str, score_thresh: float, device: str = "cuda"):
        """!
        @brief 설정/체크포인트를 로드하고 모델을 device에 올려 eval 모드로 준비한다.

        cfg_path/ckpt_path는 이 파일(model_adapter.py)이 있는 패키지 디렉토리
        기준 상대경로로 해석된다. base config(`_BASE_CONFIG_`)가 있고 상대경로면
        같은 기준으로 절대경로로 보정한다(배포용 설정은 base config 없이 단일 파일).

        @param cfg_path 모델 설정 yaml 경로 (패키지 디렉토리 기준 상대경로).
        @param ckpt_path 체크포인트(.pth) 경로 (패키지 디렉토리 기준 상대경로).
        @param score_thresh predict()에서 결과를 채택할 최소 신뢰도.
        @param device 추론에 사용할 디바이스 (기본 "cuda").
        """
        ## 추론에 사용할 디바이스 ("cuda" 또는 "cpu")
        self.device = device
        ## predict()에서 결과를 채택할 최소 신뢰도
        self.score_thresh = score_thresh
        ## OpenPCDet 모델 로드/추론 로그 출력용 logger
        self.logger = common_utils.create_logger()

        _PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
        cfg_path = os.path.join(_PACKAGE_DIR, cfg_path)
        ckpt_path = os.path.join(_PACKAGE_DIR, ckpt_path)

        # ---- config 로드 (base config 상대경로를 절대경로로 보정) ----
        with open(cfg_path, 'r') as f:
            try:
                new_config = yaml.safe_load(f, Loader=yaml.FullLoader)
            except Exception:
                new_config = yaml.safe_load(f)

        # self-contained deployment configs have no _BASE_CONFIG_; a relative one is resolved against the package dir
        base_cfg = new_config['DATA_CONFIG'].get('_BASE_CONFIG_')
        if base_cfg and not os.path.isabs(base_cfg):
            new_config['DATA_CONFIG']['_BASE_CONFIG_'] = os.path.join(_PACKAGE_DIR, base_cfg)


        merge_new_config(config=cfg, new_config=new_config)
        ## 클래스 이름 목록 (모델 출력 class_id → 이름 매핑에 사용)
        self.class_names = cfg.CLASS_NAMES

        ## 파일 스캔 없이 전처리 파이프라인만 재사용하는 경량 데이터셋
        self._dataset = _InferenceDataset(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=self.class_names,
            training=False,
        )

        ## 로드된 OpenPCDet 모델 (eval 모드, self.device에 배치됨)
        self.model = build_network(
            model_cfg=cfg.MODEL,
            num_class=len(self.class_names),
            dataset=self._dataset,
        )
        self.model.load_params_from_file(filename=ckpt_path, logger=self.logger, to_cpu=True)
        self.model.to(self.device)
        self.model.eval()

        ## 모델이 학습한 입력 점 특징 수 (KRISO VoxelNeXt: 3 = x, y, z; intensity 미사용).
        #  OpenPCDet 의 point feature encoder 가 points.shape[1] == len(src_feature_list) 를 요구한다.
        self.num_point_features = len(cfg.DATA_CONFIG.POINT_FEATURE_ENCODING.src_feature_list)
        ## 상자 자유도: 7 = [x, y, z, dx, dy, dz, yaw], 9 = [..., roll, pitch] (tilt 헤드)
        self.box_dof = int(cfg.DATA_CONFIG.get('BOX_DOF', 7))

    @torch.no_grad()
    def predict(self, points: np.ndarray) -> list[dict]:
        """!
        @brief 포인트클라우드 1프레임에 대한 3D 객체 탐지 추론.

        1. **입력 열 선택**: 모델이 학습한 점 특징 수(num_point_features, 보통 3 = x, y, z)만 남긴다.
        2. **전처리**: `_dataset.prepare_data()`/`collate_batch()`로 voxelize 등 OpenPCDet 전처리를 그대로 재사용한다.
        3. **추론**: 모델 forward, GPU에서 pred_boxes/pred_scores/pred_labels를 얻는다.
        4. **score threshold 필터링**: 생성 시 고정된 score_thresh 이상인 결과만 남긴다.
        5. **필드 변환**: OpenPCDet 박스(x,y,z,dx,dy,dz,yaw[,roll,pitch]) → Object3D 스키마 dict로 변환한다.
           7열 모델이면 roll/pitch 는 0. `pred_labels`는 1부터 시작하므로 `class_names` 인덱스에 맞추기 위해 1을 뺀다.

        @param points (N, >= 3) float32 numpy 배열 [x, y, z, (intensity, ...)]. 앞의 num_point_features 열만 사용.
        @return [{center, size, yaw, roll, pitch, class_id, class_name, score}, ...] 형태의 dict 리스트.
        """
        if points.shape[1] < self.num_point_features:
            raise ValueError(f"model expects {self.num_point_features} point features, got {points.shape[1]}")
        points = np.ascontiguousarray(points[:, :self.num_point_features], dtype=np.float32)

        # 0. 빈 프레임 보호: 점이 없거나(센서 끊김, 전부 범위 밖) 복셀이 0개면 spconv 커널 실행에서 예외가 나므로 빈 결과를 돌려준다.
        #    (복셀이 1개 이상이면 500개 미만이어도 정상 동작한다 -- centernet_utils._topk_1d 가 K 로 패딩함)
        if points.shape[0] == 0:
            return []

        # 1. 전처리 (voxelize 포함, pcdet 기능 재사용)
        data_dict = self._dataset.prepare_data({"points": points, "frame_id": 0})
        if data_dict.get("voxel_coords") is None or len(data_dict["voxel_coords"]) == 0:
            return []
        data_dict = self._dataset.collate_batch([data_dict])
        load_data_to_gpu(data_dict)

        # 2. 추론
        pred_dicts, _ = self.model.forward(data_dict)
        pred = pred_dicts[0]

        # 3. score threshold 필터링 (생성 시 고정된 값 사용)
        mask = pred["pred_scores"] >= self.score_thresh
        pred_boxes = pred["pred_boxes"][mask].cpu().numpy()    
        pred_scores = pred["pred_scores"][mask].cpu().numpy()
        pred_labels = pred["pred_labels"][mask].cpu().numpy()   

        # 4. Object3D 필드 스키마로 변환
        detections = []
        for box, score, label in zip(pred_boxes, pred_scores, pred_labels):
            x, y, z, dx, dy, dz, yaw = box[:7]
            roll, pitch = (float(box[7]), float(box[8])) if box.shape[0] >= 9 else (0.0, 0.0)
            class_id = int(label) - 1
            detections.append({
                "center": (float(x), float(y), float(z)),
                "size": (float(dx), float(dy), float(dz)),
                "yaw": float(yaw),
                "roll": roll,
                "pitch": pitch,
                "class_id": class_id,
                "class_name": self.class_names[class_id],
                "score": float(score),
            })

        return detections