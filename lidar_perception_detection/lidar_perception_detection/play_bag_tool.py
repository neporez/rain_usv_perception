## @file
#  @brief 조각(split) 번호나 시각으로 시작 지점을 정해 rosbag2 bag 을 재생하는 도구 (play_bag).

"""!
@brief 1 GB 단위로 나뉜 rosbag2 bag 을 조각 번호 / 시각 / 초 단위 offset 중 편한 것으로 시작 지점을 정해 재생한다.

`ros2 bag play` 는 폴더를 주면 모든 조각(_0, _1, ...)을 이어서 재생하지만 시작 지점은 초 단위 `--start-offset` 으로만 줄 수 있다.
이 도구는 metadata.yaml 에서 각 조각의 시작 시각을 읽어 offset 으로 바꾼 뒤 `ros2 bag play` 를 그대로 실행한다.

    ros2 run lidar_perception_detection play_bag <bag 폴더> --list                 # 조각별 시작 offset / 시각 표
    ros2 run lidar_perception_detection play_bag <bag 폴더> --file 2               # 조각 _2 의 시작부터 끝까지
    ros2 run lidar_perception_detection play_bag <bag 폴더> --time 12:52:30        # 그 시각(PC 로컬 시간)부터
    ros2 run lidar_perception_detection play_bag <bag 폴더> --offset 1300          # 1300 s 지점부터 (= ros2 bag play --start-offset)
    ros2 run lidar_perception_detection play_bag <bag 폴더> --file 2 -- --loop --rate 0.5   # `--` 뒤는 ros2 bag play 에 그대로 전달
"""
import argparse
import datetime as dt
import os
import subprocess
import sys

import yaml


def load_files(bag):
    """!
    @brief metadata.yaml 에서 조각 목록과 시작 시각을 읽는다.
    @param bag bag 폴더.
    @return (bag 시작 시각 ns, [(path, 시작 offset s, 길이 s), ...]).
    """
    meta = os.path.join(bag, "metadata.yaml")
    if not os.path.isfile(meta):
        sys.exit(f"metadata.yaml not found in {bag} (give the bag FOLDER, not one .mcap file)")
    m = yaml.safe_load(open(meta))["rosbag2_bagfile_information"]
    t0 = m["starting_time"]["nanoseconds_since_epoch"]
    files = m.get("files") or []
    if len(files) != len(m["relative_file_paths"]) or len({f["starting_time"]["nanoseconds_since_epoch"] for f in files}) < len(files):
        files = _scan_pieces(bag, m["relative_file_paths"], m["duration"]["nanoseconds"])      # old / reindexed metadata: no per-piece times
    rows = [(f["path"], (f["starting_time"]["nanoseconds_since_epoch"] - t0) / 1e9, f["duration"]["nanoseconds"] / 1e9) for f in files]
    return t0, sorted(rows, key=lambda r: r[1])


def _scan_pieces(bag, paths, total_ns):
    """!
    @brief 조각별 시작 시각이 metadata 에 없을 때(옛 형식, `ros2 bag reindex` 결과) 각 조각 파일을 직접 열어 첫 메시지 시각을 읽는다.
    @param bag bag 폴더.
    @param paths metadata 의 relative_file_paths.
    @param total_ns bag 전체 길이 (ns).
    @return metadata 의 files 항목과 같은 형식의 리스트.
    """
    import rosbag2_py
    out = []
    for p in paths:
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=os.path.join(bag, p), storage_id=""), rosbag2_py.ConverterOptions("", ""))
        start = r.read_next()[2] if r.has_next() else None
        out.append({"path": p, "starting_time": {"nanoseconds_since_epoch": start}, "duration": {"nanoseconds": 0}})
    out = [f for f in out if f["starting_time"]["nanoseconds_since_epoch"] is not None]
    out.sort(key=lambda f: f["starting_time"]["nanoseconds_since_epoch"])
    end = out[0]["starting_time"]["nanoseconds_since_epoch"] + total_ns if out else 0
    for f, nxt in zip(out, out[1:] + [None]):
        f["duration"]["nanoseconds"] = (nxt["starting_time"]["nanoseconds_since_epoch"] if nxt else end) - f["starting_time"]["nanoseconds_since_epoch"]
    return out


def main(argv=None):
    """!
    @brief 명령행 진입점: 시작 지점을 offset 으로 바꿔 ros2 bag play 를 실행한다.
    @param argv 명령행 인자 (None 이면 sys.argv).
    @return ros2 bag play 의 종료 코드.
    """
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag", help="rosbag2 folder (holds metadata.yaml and the _0, _1, ... pieces)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--file", type=int, metavar="N", help="start at piece _N (index in the metadata file list)")
    g.add_argument("--time", metavar="HH:MM:SS", help="start at this local wall-clock time of the recording day")
    g.add_argument("--offset", type=float, metavar="SEC", help="start SEC seconds after the bag start")
    ap.add_argument("--list", action="store_true", help="print the pieces with their start offsets and times, do not play")
    ap.epilog = "arguments after '--' are passed to ros2 bag play unchanged (e.g. -- --loop --rate 0.5 --topics /dev/lidar/lidar_packets)"
    argv = list(sys.argv[1:] if argv is None else argv)
    rest = []
    if "--" in argv:
        rest = argv[argv.index("--") + 1:]; argv = argv[:argv.index("--")]
    a = ap.parse_args(argv)
    bag = os.path.abspath(a.bag)
    t0, rows = load_files(bag)
    start = dt.datetime.fromtimestamp(t0 / 1e9)
    if a.list or (a.file is None and a.time is None and a.offset is None):
        print(f"{bag}\nbag start {start:%Y-%m-%d %H:%M:%S} local, {len(rows)} piece(s)")
        for i, (p, off, dur) in enumerate(rows):
            print(f"  --file {i:<3d} {p:<45s} offset {off:8.1f} s  {start + dt.timedelta(seconds=off):%H:%M:%S} ~ {start + dt.timedelta(seconds=off + dur):%H:%M:%S}")
        if a.list or not rest:
            return 0
    if a.file is not None:
        if not 0 <= a.file < len(rows):
            sys.exit(f"--file must be 0..{len(rows) - 1}")
        offset = rows[a.file][1]
    elif a.time is not None:
        h, mi, s = (list(map(float, a.time.split(":"))) + [0, 0])[:3]
        target = start.replace(hour=int(h), minute=int(mi), second=int(s), microsecond=0)
        offset = (target - start).total_seconds()
        if offset < 0 or offset > rows[-1][1] + rows[-1][2]:
            sys.exit(f"{a.time} is outside the recording ({start:%H:%M:%S} ~ {start + dt.timedelta(seconds=rows[-1][1] + rows[-1][2]):%H:%M:%S})")
    else:
        offset = a.offset or 0.0
    cmd = ["ros2", "bag", "play", bag, "--start-offset", f"{max(offset, 0.0):.1f}"] + rest
    print(f"start at +{offset:.1f} s = {start + dt.timedelta(seconds=offset):%H:%M:%S} local\n$ {' '.join(cmd)}", flush=True)
    return subprocess.call(cmd)


if __name__ == "__main__":
    sys.exit(main())
