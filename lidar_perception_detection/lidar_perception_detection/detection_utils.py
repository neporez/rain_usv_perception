## @file
#  @brief PointCloud2 파싱, QoS 변환 등 detection_node 보조 함수 모음.

"""! @brief PointCloud2 파싱, QoS 변환 등 detection_node 보조 함수 모음. """
import numpy as np
from sensor_msgs.msg import PointCloud2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from lidar_perception_interfaces.msg import Object3D, Object3DArray

def resolve_qos(reliability: str, depth: int = 1) -> QoSProfile:
    """!
    @brief Config 문자열('best_effort' | 'reliable')을 QoSProfile로 변환.

    @param reliability "best_effort" 또는 "reliable" 중 하나.
    @param depth 큐 depth (KEEP_LAST 기준 보관할 메시지 개수).
    @return 구성된 QoSProfile.
    """
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
    """!
    @brief sensor_msgs/PointCloud2 → (N, 4) float32 numpy [x, y, z, intensity] 변환.

    필드 offset을 메시지 헤더(msg.fields)에서 자동으로 읽어 structured dtype을
    구성하기 때문에, 필드 순서가 바뀌어도 안전하게 파싱된다. NaN/Inf를 포함한
    포인트는 이후 처리(모델 추론)에 방해가 되므로 결과에서 제거한다.

    @param msg 원본 sensor_msgs/PointCloud2 메시지. x, y, z, intensity 필드가 있어야 한다.
    @return (N, 4) float32 numpy 배열 [x, y, z, intensity]. NaN/Inf 포인트는 제외됨.
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