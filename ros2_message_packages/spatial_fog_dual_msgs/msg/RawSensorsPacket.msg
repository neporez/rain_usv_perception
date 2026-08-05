# Raw Sensors Packet (Packet ID 28)

std_msgs/Header header

# Accelerometer
float64 raw_lin_acc_x       # m/s2
float64 raw_lin_acc_y       # m/s2
float64 raw_lin_acc_z       # m/s2

# Gyroscope
float64 raw_gyro_x      # deg/s
float64 raw_gyro_y      # deg/s
float64 raw_gyro_z      # deg/s

# Magnetometer
float64 raw_mag_x       # mG    milligauss
float64 raw_mag_y       # mG
float64 raw_mag_z       # mG

# ETC
float64 raw_imu_temperature         # deg C
float64 raw_pressure                # Pascals
float64 raw_pressure_temperature    # deg C


