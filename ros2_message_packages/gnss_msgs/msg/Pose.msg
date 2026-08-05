# (1) 시간은 Epoch Time을 표준으로 함
# (2) 위경도 값은 십진수 값으로 사용함
# (3) 거리, 속도는 m, deg, m/s, deg/sec을 사용함
# (4) 글자수에 관계없이 모든 문자와 문자열은 string으로 사용함

# ROS
std_msgs/Header header

# Time from GPRMC
float64 gnss_time                   # UNIX TIME 

# Position as LLA 
float64 latitude                    # 십진수, 북쪽이 양수, 남쪽이 음수, -90 ~ +90
float64 longitude                   # 십진수, 동경이 양수, 서경이 음수, -180 ~ +180
float64 altitude                    # 단위 m

# Position as UTM
float64 utmx                        # m
float64 utmy                        # m
string utmzone        

# Orientation 
float64 roll                        # deg, 수평 0도, 우현이 하강하면 양의방향 (전진 x축)
float64 pitch                       # deg, 수평 0도, 기수가 들리면 양의방향 (우측 y축)
float64 yaw                         # deg, 진북 0도, 시계방향으로 증가, 0 ~ 360 범위

# cog, sog 
float64 sog                         # m/s
float64 cog                         # deg


