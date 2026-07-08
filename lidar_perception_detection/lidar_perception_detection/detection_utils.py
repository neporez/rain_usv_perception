import numpy as np
from sensor_msgs.msg import PointCloud2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from lidar_perception_interfaces.msg import Object3D, Object3DArray

def resolve_qos(reliability: str, depth: int = 1) -> QoSProfile:
    """Config 문자열('best_effort' | 'reliable')을 QoSProfile로 변환"""
    profile = QoSProfile(depth=depth)
    profile.history = QoSHistoryPolicy.KEEP_LAST

    if reliability == "best_effort":
        profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
    elif reliability == "reliable":
        profile.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        raise ValueError(f"Unknown reliability setting: {reliability}")

    return profile


def parse_pointcloud2(msg: PointCloud2) -> np.ndarray:
    """
    sensor_msgs/PointCloud2 → (N, 4) float32 numpy [x, y, z, intensity].
    필드 offset을 메시지 헤더에서 자동으로 읽어 structured dtype을 구성한다.
    NaN/Inf를 포함한 포인트는 제거한다.
    """
    field_map = {f.name: f for f in msg.fields}

    missing = [n for n in ("x", "y", "z", "intensity") if n not in field_map]
    if missing:
        raise ValueError(f"PointCloud2 message missing expected fields: {missing}")

    dtype = np.dtype({
        "names": ["x", "y", "z", "intensity"],
        "formats": [np.float32, np.float32, np.float32, np.float32],
        "offsets": [
            field_map["x"].offset,
            field_map["y"].offset,
            field_map["z"].offset,
            field_map["intensity"].offset,
        ],
        "itemsize": msg.point_step,
    })

    cloud = np.frombuffer(bytes(msg.data), dtype=dtype)
    xyzi = np.column_stack([cloud["x"], cloud["y"], cloud["z"], cloud["intensity"]])

    valid = np.isfinite(xyzi).all(axis=1)
    return xyzi[valid].astype(np.float32)