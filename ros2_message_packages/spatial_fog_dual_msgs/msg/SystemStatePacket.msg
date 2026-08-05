# System State Packet (Packet ID 20) 

std_msgs/Header header

# System status
int64 system_status               # check manual
int64 filter_status               # check manaul
int64 unix_time_seconds           
int64 unix_time_micro_seconds

# LLA
float64 latitude                # degree (raw sensor output is radian)
float64 longitude               # degree (raw sensor output is radian)
float64 altitude                # degree (raw sensor output is radian)

# NED Velocity
float64 vel_north               # m/s
float64 vel_east                # m/s
float64 vel_down                # m/s

# Body Acceleration
float64 body_acc_x              # m/s/s
float64 body_acc_y              # m/s/s
float64 body_acc_z              # m/s/s

# G force
float64 g_force                 # g

# Euler Angle
float64 roll            # deg
float64 pitch           # deg
float64 yaw             # deg

# Angular Velocity  
float64 ang_vel_x       # deg/s (raw sensor output is rad/sec)            
float64 ang_vel_y       # deg/s (raw sensor output is rad/sec)
float64 ang_vel_z       # deg/s (raw sensor output is rad/sec)

# Euler Angle Standard Variation
float64 lat_std_dev     # m
float64 lon_std_dev     # m
float64 alt_std_dev     # m
