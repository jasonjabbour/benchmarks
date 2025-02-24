std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max

# Example approach: store ranges as int16 and intensities as uint8
# We'll also store "scale" parameters to help interpret those fields
int16[] ranges
uint8[] intensities

float32 range_scale       # how to decode int16 ranges => real range
float32 intensity_scale   # how to decode uint8 intensities => real intensities