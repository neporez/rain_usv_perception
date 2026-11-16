## @file
#  @brief Ouster 원시 패킷(ouster_sensor_msgs/PacketMsg) → 프레임 점군 디코더 (ouster-sdk >= 1.0).

"""!
@brief Ouster 원시 패킷(ouster_sensor_msgs/PacketMsg) → 프레임 점군 디코더 (ouster-sdk >= 1.0).

드라이버(ouster_ros os_driver, 설정: point_type original, organized, destagger, point_cloud_frame os_lidar,
timestamp_mode TIME_FROM_ROS_TIME)가 발행하던 /dev/lidar/points 와 같은 점(x, y, z, intensity, reflectivity, ring, range)을
패킷에서 복원한다. 조직화(organized)되지 않은 유효 점(range > 0)만 내보내며, 좌표계는 기본값 'lidar'(os_lidar =
드라이버 출력과 동일: 센서 좌표를 lidar_to_sensor_transform 의 역으로 변환) 또는 'sensor'(os_sensor).
프레임 시각(stamp_mode):
  'driver'     ouster_ros 드라이버(TIME_FROM_ROS_TIME)와 동일 = **직전 프레임**의 첫 패킷 수신 시각. 드라이버는 프레임 N 의
               점군에 프레임 N-1 의 첫 패킷 시각을 찍는다(2026-11-06 bag 으로 확인: 데이터셋 프레임 lidar_ns 와 정확히 100 ms 차이,
               점 일치 99 %). 학습 데이터·GNSS 보간이 이 규약으로 만들어졌으므로 기본값.
  'scan_start' 이 프레임의 첫 패킷 수신 시각(물리적으로 맞는 값, 드라이버보다 100 ms 늦음).
"""
import numpy as np

try:
    from ouster.sdk import core as _osc
except ImportError as e:   # pragma: no cover
    raise ImportError("ouster-sdk >= 1.0 is required for packet input: pip install ouster-sdk") from e


class OusterPacketDecoder:
    """!
    @brief 라이다 패킷을 하나씩 받아 프레임(2048열)이 완성될 때마다 점군을 돌려주는 디코더.

    ouster-sdk 의 FrameBatcher 로 패킷을 LidarFrame 에 모으고, XYZLut 으로 range 이미지를 센서 좌표로 바꾼 뒤
    range > 0 인 유효 점만 남긴다. 좌표계는 드라이버(ouster_ros, point_cloud_frame os_lidar)와 같게 lidar_to_sensor_transform 의
    역을 적용한다(평행이동은 metadata 에 mm 단위). 프레임 시각 규약은 stamp_mode 참조.
    """

    def __init__(self, metadata_json: str, point_frame: str = "lidar", stamp_mode: str = "driver"):
        """!
        @brief 센서 metadata(JSON)로 디코더를 구성.
        @param metadata_json 드라이버가 /dev/lidar/metadata 로 발행하는(또는 파일로 저장한) 센서 metadata JSON 문자열.
        @param point_frame 'lidar'(os_lidar, 드라이버 출력과 동일) | 'sensor'(os_sensor).
        @param stamp_mode 'driver'(직전 프레임 첫 패킷 시각 = ouster_ros 규약) | 'scan_start'(이 프레임 첫 패킷 시각).
        """
        if point_frame not in ("lidar", "sensor"):
            raise ValueError("point_frame must be 'lidar' or 'sensor'")
        if stamp_mode not in ("driver", "scan_start"):
            raise ValueError("stamp_mode must be 'driver' or 'scan_start'")
        self.stamp_mode = stamp_mode
        self._prev_first_host_ns = None
        self.info = _osc.SensorInfo(metadata_json)
        self.pf = _osc.PacketFormat(self.info)
        self.lidar_packet_size = int(self.pf.lidar_packet_size)
        self.batcher = _osc.FrameBatcher(self.info)
        self.lut = _osc.XYZLut(self.info)
        self.point_frame = point_frame
        T = np.array(self.info.lidar_to_sensor_transform, dtype=np.float64).reshape(4, 4)   # lidar -> sensor, translation in mm
        T[:3, 3] /= 1000.0                                                                      # -> metres (OS-2: z +0.0783 m)
        self._T_s2l = np.linalg.inv(T)                                                         # sensor -> lidar
        self.h, self.w = int(self.info.format.pixels_per_column), int(self.info.format.columns_per_frame)
        self._frame = _osc.LidarFrame(self.info)
        self._first_host_ns = None
        self.n_packets = 0
        self.n_frames = 0
        self.n_bad_size = 0

    # ------------------------------------------------------------------
    def describe(self) -> str:
        """!
        @brief 센서 종류 / 모드 / 프레임 크기 / 출력 좌표계 / 시각 규약을 한 줄로 요약.
        @return 로그용 문자열.
        """
        si = self.info
        return (f"Ouster {si.prod_line} sn {si.sn} mode {si.config.lidar_mode} profile {si.format.udp_profile_lidar} "
                f"({self.h} x {self.w}, {self.lidar_packet_size} B/packet), output frame {self.point_frame}, stamp {self.stamp_mode}")

    def feed(self, buf, host_time_ns: int):
        """!
        @brief 라이다 패킷 하나를 넣는다. 프레임이 완성되면 점군 dict 를, 아니면 None 을 돌려준다.
        @param buf 패킷 바이트열 (PacketMsg.buf). 길이가 lidar_packet_size 와 다르면 버린다.
        @param host_time_ns 패킷 수신 호스트 시각 [ns] (프레임 시각 계산에 사용).
        @return dict(xyz (N,3) float32, intensity, reflectivity, ring, range, stamp_ns, frame_id, n_valid_cols) 또는 None.
        """
        data = np.frombuffer(bytes(buf), dtype=np.uint8) if not isinstance(buf, np.ndarray) else buf
        if data.size != self.lidar_packet_size:
            self.n_bad_size += 1
            return None
        pkt = _osc.LidarPacket(self.pf)
        pkt.buf[:] = data
        try:
            pkt.host_timestamp = int(host_time_ns)
        except (AttributeError, TypeError):
            pass
        if self._first_host_ns is None:
            self._first_host_ns = int(host_time_ns)
        self.n_packets += 1
        if not self.batcher(pkt, self._frame):
            return None
        if self.stamp_mode == "driver":
            stamp = self._prev_first_host_ns if self._prev_first_host_ns is not None else self._first_host_ns - 100_000_000
        else:
            stamp = self._first_host_ns
        out = self._convert(self._frame, stamp)
        self._prev_first_host_ns = self._first_host_ns
        self._frame = _osc.LidarFrame(self.info)
        self._first_host_ns = None
        self.n_frames += 1
        return out

    def _convert(self, frame, stamp_ns):
        """!
        @brief 완성된 LidarFrame 을 유효 점 배열로 변환 (XYZLut → 좌표계 변환 → 필드 추출).
        @param frame ouster-sdk LidarFrame.
        @param stamp_ns 프레임 시각 [ns].
        @return feed() 가 돌려주는 dict.
        """
        rng = np.asarray(frame.field(_osc.ChanField.RANGE))                 # (H, W) uint32 mm, staggered
        xyz = np.asarray(self.lut(rng)).reshape(-1, 3)                       # sensor frame, metres
        valid = rng.reshape(-1) > 0
        xyz = xyz[valid]
        if self.point_frame == "lidar":
            xyz = xyz @ self._T_s2l[:3, :3].T + self._T_s2l[:3, 3]
        ring = np.repeat(np.arange(self.h, dtype=np.uint16), self.w)[valid]
        sig = np.asarray(frame.field(_osc.ChanField.SIGNAL)).reshape(-1)[valid]
        refl = np.asarray(frame.field(_osc.ChanField.REFLECTIVITY)).reshape(-1)[valid]
        return {"xyz": xyz.astype(np.float32), "intensity": sig.astype(np.float32), "reflectivity": refl.astype(np.uint16),
                "ring": ring, "range": rng.reshape(-1)[valid].astype(np.uint32), "stamp_ns": int(stamp_ns),
                "frame_id": int(frame.frame_id), "n_valid_cols": int(np.count_nonzero(np.asarray(frame.status) & 1))}
