## @file
#  @brief rosbag2 bag 에서 Ouster 센서 metadata(JSON)를 파일로 꺼내는 도구 (extract_ouster_metadata).

"""!
@brief rosbag2 bag 에서 Ouster 센서 metadata(JSON, /dev/lidar/metadata std_msgs/String)를 파일로 꺼낸다.

ouster_packet_node 는 패키지의 config/ouster_metadata.json (OS-2-128 SN 992203000509, 2048x10) 으로 시작하므로 bag 을
중간부터 재생해도 metadata 없이 동작한다. 센서(일련번호)나 모드(1024x20 등)가 바뀌면 이 도구로 새 bag 에서 JSON 을 뽑아
그 파일을 교체하거나 `metadata_path` 파라미터(launch 인자 `ouster_metadata_path:=...`)로 넘긴다. 실제 센서 운용 시에는
드라이버의 latched 토픽이 파일보다 우선한다.

    ros2 run lidar_perception_detection extract_ouster_metadata <bag_dir> [-o lidar_metadata.json] [--topic /dev/lidar/metadata]
"""
import argparse
import json
import sys


def main(argv=None):
    """!
    @brief 명령행 진입점: bag 의 첫 metadata 메시지를 읽어 JSON 파일로 저장.
    @param argv 명령행 인자 (None 이면 sys.argv).
    @return 0 성공, 1 메시지 없음.
    """
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag", help="rosbag2 directory (mcap or sqlite3)")
    ap.add_argument("-o", "--out", default="lidar_metadata.json")
    ap.add_argument("--topic", default="/dev/lidar/metadata")
    a = ap.parse_args(argv)
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[a.topic]))
    if not reader.has_next():
        print(f"no message on {a.topic} in {a.bag}", file=sys.stderr)
        return 1
    _, data, _ = reader.read_next()
    txt = deserialize_message(data, String).data
    meta = json.loads(txt)                                  # validate
    with open(a.out, "w") as f:
        f.write(txt)
    si = meta.get("sensor_info", {})
    print(f"wrote {a.out}: {si.get('prod_line', '?')} sn {si.get('prod_sn', '?')} mode {meta.get('config_params', {}).get('lidar_mode', '?')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
