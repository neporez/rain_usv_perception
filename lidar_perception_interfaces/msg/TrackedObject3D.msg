# 트래킹된 객체 하나를 표현하는 메시지

Object3D detection              # 이 트랙에 매칭된(혹은 예측된) 현재 프레임의 3D 검출 정보

int32 track_id                  # 트랙 고유 ID, 트랙이 유지되는 동안 프레임 간 동일하게 유지됨
int32 age                       # 트랙 생존 기간, 트랙 생성 후 유지 기간                                

geometry_msgs/Vector3 velocity  # 추정된 속도 [vx, vy, vz] (m/s)
geometry_msgs/Point global_position  # 전역(world) 좌표계 기준 위치 [x, y, z] (m)
  
float64 global_yaw               # 전역 좌표계 기준 헤딩 (rad)  
