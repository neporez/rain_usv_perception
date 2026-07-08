"""
model_adapter.py

ROS2 detection_node가 아는 건 이 파일의 VoxelNeXtInferencer 클래스뿐이다.
pcdet 관련 import/로직은 전부 이 파일 안에서만 다룬다.
"""
import numpy as np
import torch
import yaml

from pcdet.config import cfg, merge_new_config
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils

import os


class _InferenceDataset(DatasetTemplate):
    """
    파일 스캔 없이 prepare_data()/collate_batch()만 재사용하기 위한
    경량 DatasetTemplate 서브클래스.
    """

    def __init__(self, dataset_cfg, class_names, training=False):
        super().__init__(
            dataset_cfg=dataset_cfg,
            class_names=class_names,
            training=training,
            root_path=None,
            logger=None,
        )

    def __len__(self):
        return 0

    def __getitem__(self, index):
        raise NotImplementedError


class VoxelNeXtInferencer:
    def __init__(self, cfg_path: str, ckpt_path: str, score_thresh: float, device: str = "cuda"):
        self.device = device
        self.score_thresh = score_thresh
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

        base_cfg = new_config['DATA_CONFIG']['_BASE_CONFIG_']
        if base_cfg and not os.path.isabs(base_cfg):
            new_config['DATA_CONFIG']['_BASE_CONFIG_'] = os.path.join(_PACKAGE_DIR, base_cfg)
            

        merge_new_config(config=cfg, new_config=new_config)
        self.class_names = cfg.CLASS_NAMES 

        self._dataset = _InferenceDataset(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=self.class_names,
            training=False,
        )

        self.model = build_network(
            model_cfg=cfg.MODEL,
            num_class=len(self.class_names),
            dataset=self._dataset,
        )
        self.model.load_params_from_file(filename=ckpt_path, logger=self.logger, to_cpu=True)
        self.model.to(self.device)
        self.model.eval()

    @torch.no_grad()
    def predict(self, points: np.ndarray) -> list[dict]:
        """
        points: (N, 4) float32 numpy [x, y, z, intensity]
        return: [{center, size, yaw, class_id, class_name, score}, ...]
        """
        # 1. 전처리 (voxelize 포함, pcdet 기능 재사용)
        data_dict = self._dataset.prepare_data({"points": points, "frame_id": 0})
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
            x, y, z, dx, dy, dz, yaw = box
            class_id = int(label) - 1
            detections.append({
                "center": (float(x), float(y), float(z)),
                "size": (float(dx), float(dy), float(dz)),
                "yaw": float(yaw),
                "class_id": class_id,
                "class_name": self.class_names[class_id],
                "score": float(score),
            })

        return detections