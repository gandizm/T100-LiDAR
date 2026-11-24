# Livox-compatible custom point cloud schema. Layout follows Livox ROS Driver2 (MIT).
# Copyright (c) 2022 Livox. All rights reserved.
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
