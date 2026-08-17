# 3D 바운딩 박스 하나를 표현하는 메시지

geometry_msgs/Point center      # 박스 중심 좌표 [x, y, z] (m), 기준 프레임(frame_id)은 헤더 또는 상위 메시지에서 정의
geometry_msgs/Vector3 size      # 박스 크기 [x: length, y: width, z: height] (m)
float64 yaw                     # 박스의 헤딩(회전) 각도 (rad), 보통 x축 기준 반시계 방향, [-pi, pi) 범위

int32 class_id                  # 클래스 인덱스 (예: 0=boat, ...), 모델의 CLASS_NAMES 순서와 매핑됨
string class_name               # class_id에 대응하는 사람이 읽을 수 있는 클래스 이름 (예: "boat")

float64 score                   # 검출 신뢰도(confidence), [0, 1] 범위
