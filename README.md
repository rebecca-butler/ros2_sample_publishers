# ros2_sample_publishers

This repository contains nodes for publishing messages with different types.
This can be used to debug issues in RViz or other ROS 2 systems.

## Contents
- [path_pub](src/path_publisher.cpp)
    - publish a `nav_msgs/msg/Path` path and `geometry_msgs::msg::Transform` transform
- [point_cloud_pub](src/point_cloud_publisher.cpp)
    - publish a `sensor_msgs::msg::PointCloud2` point cloud

## Build
```bash
$ colcon build --packages-select ros2_sample_publishers
```

## Usage example
```
$ source /opt/ros/galactic/setup.bash
$ ros2 run ros2_sample_publishers point_cloud_pub
```
