# (1) 시간은 Epoch Time을 표준으로 함
# (2) 위경도 값은 십진수 값으로 사용함
# (3) 거리, 속도는 m, deg, m/s, deg/sec을 사용함
# (4) 글자수에 관계없이 모든 문자와 문자열은 string으로 사용함

# ROS
std_msgs/Header header

# From GPRMC
float64 gnss_time               # UNIX TIME 

# LLA 
float64 latitude                    # 십진수, 북쪽이 양수, 남쪽이 음수, -90 ~ +90
float64 longitude                   # 십진수, 동경이 양수, 서경이 음수, -180 ~ +180
float64 altitude                    # 단위 m

# UTM
float64 utmx                        # m
float64 utmy                        # m
string utmzone        

# cog, sog 
float64 sog                         # m/s
float64 cog                         # deg

# PSAT,HPR
float64 roll                        # deg, 수평 0도, 우현이 하강하면 양의방향 (전진 x축)
float64 pitch                       # deg, 수평 0도, 기수가 들리면 양의방향 (우측 y축)
float64 yaw                         # deg, 진북 0도, 시계방향으로 증가, 0 ~ 360 범위
string heading_type                 # N : GNSS derived heading / G : gyro heading

# ETC GGA
int32 gnss_quality                  # 0 : no position, # 1 : GPS Fix, # 2 : DGPS fix, # 4 : RTK, # 5 : Float float
int32 num_of_satellite              # 위성 수
float64 hdop                        # 낮을수록 정밀도가 낮음, 이론적으로 0은 나올수 없음, 파싱오류 발생 시 0 사용
float64 geoidal_separation          # wgs84 지구 타원체와 실제 해수면(지오이드) 사이의 높이 차이, 단위는 m임
float64 age_of_diff_corrections     # 마지막으로 차동 보정 데이터를 받은 이후로 경과한 시간
int32 diff_ref_station_id           # 차동 참조국 식별자

# ETC RMC
float64 mag_var                     # magnetic variation, 진북과 자북이 동일할 때 0, 자북이 더 동쪽일때 양, 자북이 더 서쪽일때 음
string mode_indicator               # N, A, D, P, R, F, E, M, S : 자세한 내용은 매뉴얼 참조
string navigational_status          # S, C, U, V : 자세한 내용은 매뉴얼 참조




              