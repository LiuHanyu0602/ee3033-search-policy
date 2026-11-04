# Original detector image timestamp and calibrated optical frame.
std_msgs/Header header
# Unique per configured search; invalidates all results from prior searches.
string episode_id
int32 class_id
string class_name
float64 confidence
string tracking_id
# Axis-aligned bbox in original detector image pixels.
float64 center_x
float64 center_y
float64 width
float64 height
uint32 image_width
uint32 image_height
# True only after fresh distinct-frame confirmation.
bool confirmed
