# Public Livox ROS Driver 2 message layout; field order/types retained for compatibility.
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
