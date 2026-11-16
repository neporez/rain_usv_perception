"""
extract_ouster_metadata -- rosbag2 bag 에서 Ouster 센서 metadata(JSON, /dev/lidar/metadata std_msgs/String)를 파일로 꺼낸다.

드라이버는 metadata 를 bag 시작 시 한 번만(latched) 기록하므로 `ros2 bag play --start-offset ...` 으로 중간부터 재생하면
ouster_packet_node 가 metadata 를 받지 못한다. 그때는 이 도구로 JSON 을 뽑아 `metadata_path` 파라미터(또는 launch 인자
`ouster_metadata_path:=...`)로 넘긴다. 실제 센서 운용 시에는 드라이버의 latched 토픽으로 충분하다.

    ros2 run lidar_perception_detection extract_ouster_metadata <bag_dir> [-o lidar_metadata.json] [--topic /dev/lidar/metadata]
"""
import argparse
import json
import sys


def main(argv=None):
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
