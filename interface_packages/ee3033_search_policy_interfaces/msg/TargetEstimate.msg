# Measurement timestamp, output frame must be map.
std_msgs/Header header
string episode_id
int32 class_id
string class_name
geometry_msgs/Point position
# Row-major 2x2 covariance for x,y in square metres.
float64[4] covariance_xy
# Quality score, not a calibrated probability of correctness.
float64 quality
string method
bool valid
string reason
