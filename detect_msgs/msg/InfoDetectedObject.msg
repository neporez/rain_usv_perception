int64 id                 # 트래킹 ID
int64 target_label       # 클래스 인덱스
float64 score            # 신뢰도 (0.0 ~ 1.0)
int64 x                  # 바운딩박스 중심 x (px)
int64 y                  # 바운딩박스 중심 y (px)
int64 w                  # 바운딩박스 너비 (px)
int64 h                  # 바운딩박스 높이 (px)
float64 theta            # 수평 방위각 (rad, 오른쪽 양수)
float64 range            # 거리 (m)
float64 speed            # 속도 (m/s)
float64 heading          # 진행 방향 (rad)
float64 x_covariances
float64 y_covariances
float64 w_covariances
float64 h_covariances
float64 range_covariances
float64 theta_covariances
float64 speed_covariances
float64 heading_covariances