# 3D bounding box of one detected object in the LiDAR (sensor) frame.
geometry_msgs/Point center      # box centre [m]
geometry_msgs/Vector3 size      # length (x, along the heading), width (y), height (z) [m]
float64 yaw                     # heading about the sensor z axis [rad]
float64 roll                    # tilt about the box's own x axis [rad]; rotation R = Rz(yaw) Ry(pitch) Rx(roll); 0 for 7-DoF models
float64 pitch                   # tilt about the box's own y axis [rad]

int32 class_id                  # 0-based index into the model's class list
string class_name               # e.g. "boat"
float64 score                   # detection confidence in [0, 1]
