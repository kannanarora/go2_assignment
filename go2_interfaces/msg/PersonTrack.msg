std_msgs/Header header

bool visible
string track_id
float32 confidence

# Bearing from the robot/camera forward axis. Positive is left.
float32 bearing_rad

# Range estimated from the front LiDAR scan in the same bearing sector.
bool distance_valid
float32 distance_m

# True when distance_valid and distance_m <= nearby_threshold_m.
bool nearby

# Original 2D detection geometry in image pixels.
float32 image_x
float32 image_y
float32 bbox_width
float32 bbox_height
