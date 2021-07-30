# ros2_sample_publishers

This repository contains nodes for publishing messages with different types.
This can be used to debug issues in RViz or other systems.

## Contents
- `path_publisher.cpp`
    - publish a `nav_msgs/msg/Path` path and `geometry_msgs::msg::Transform` transform
- `point_cloud_publisher.cpp`
    - publish a `sensor_msgs::msg::PointCloud2` point cloud

## Build
```bash
$ rosdep install -i --from-path src --rosdistro galactic -y
$ source /opt/ros/galactic/setup.bash
$ colcon build
```

## Usage
```
$ ros2 run ros2_sample_publishers path_pub
$ ros2 run ros2_sample_publishers point_cloud_pub
```
