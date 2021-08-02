// Copyright 2021 Rebecca Butler
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher()
  : Node("point_cloud_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "points2", rclcpp::QoS(1).transient_local());
    timer_callback();
    timer_ = this->create_wall_timer(5s, std::bind(&PointCloudPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // create header
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Clock().now();
    header.frame_id = "/map";
    points_msg->header = header;

    // set number of test points to use
    int num_points = 100;

    // set message fields
    points_msg->height = 1;
    points_msg->width = num_points * num_points * num_points;
    points_msg->is_bigendian = false;
    points_msg->is_dense = false;  // there may be invalid points

    sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
    pcd_modifier.setPointCloud2Fields(
      3,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");

    // create cube cloud
    for (int u = 0; u < num_points; ++u) {
      for (int v = 0; v < num_points; ++v) {
        for (int w = 0; w < num_points; ++w) {
          *iter_x = u / 10;
          *iter_y = v / 10;
          *iter_z = w / 10;

          ++iter_x;
          ++iter_y;
          ++iter_z;
        }
      }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing cloud" << std::endl);
    publisher_->publish(*points_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
