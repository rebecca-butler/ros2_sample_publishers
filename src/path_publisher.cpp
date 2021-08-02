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
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher()
  : Node("path_publisher")
  {
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(1).transient_local());
    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

    path_timer_callback();
    tf_timer_callback();

    path_timer_ = this->create_wall_timer(5s, std::bind(&PathPublisher::path_timer_callback, this));
    tf_timer_ = this->create_wall_timer(5s, std::bind(&PathPublisher::tf_timer_callback, this));
  }

private:
  void path_timer_callback()
  {
    // create path message
    auto message = nav_msgs::msg::Path();
    message.header.stamp = now();
    message.header.frame_id = "odom";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";

    auto x_pos = 0.0;
    auto y_pos = 0.0;
    auto z_pos = 0.0;

    // add line along x-axis
    for (unsigned int i = 0; i < 3; i++) {
      auto pose = create_pose(
        x_pos++, y_pos, z_pos,
        0.0, 0.0, 0.0, 1.0);
      message.poses.push_back(pose);
    }

    // add line along y-axis
    for (unsigned int i = 0; i < 3; i++) {
      auto pose = create_pose(
        x_pos, y_pos++, z_pos,
        0.0, 0.0, sin(M_PI / 4), cos(M_PI / 4));
      message.poses.push_back(pose);
    }

    // add line along z-axis
    for (unsigned int i = 0; i < 3; i++) {
      auto pose = create_pose(
        x_pos, y_pos, z_pos++,
        0.0, -sin(M_PI / 4), 0.0, cos(M_PI / 4));
      message.poses.push_back(pose);
    }

    // add line between x and z axes
    for (unsigned int i = 0; i < 3; i++) {
      auto pose = create_pose(
        x_pos++, y_pos, z_pos++,
        0.0, sin(M_PI / 8), 0.0, -cos(M_PI / 8));
      message.poses.push_back(pose);
    }

    // publish message
    path_publisher_->publish(message);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing path" << std::endl);
  }

  void tf_timer_callback()
  {
    // create tf message
    tf2_msgs::msg::TFMessage message;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";

    // create tf
    auto rotation = geometry_msgs::msg::Quaternion();
    rotation.x = 0.0;
    rotation.y = 0.0;
    rotation.z = 1.0;
    rotation.w = 1.0;
    tf.transform.rotation = rotation;
    auto translation = geometry_msgs::msg::Vector3();
    translation.x = 0.0;
    translation.y = 0.0;
    translation.z = 1.0;
    tf.transform.translation = translation;

    message.transforms.push_back(tf);

    // publish message
    tf_publisher_->publish(message);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing tf" << std::endl);
  }

  geometry_msgs::msg::PoseStamped create_pose(
    const float_t & pos_x, const float_t & pos_y, const float_t & pos_z,
    const float_t & orient_x, const float_t & orient_y, const float_t & orient_z,
    const float_t & orient_w)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = pos_x;
    pose.pose.position.y = pos_y;
    pose.pose.position.z = pos_z;
    pose.pose.orientation.x = orient_x;
    pose.pose.orientation.y = orient_y;
    pose.pose.orientation.z = orient_z;
    pose.pose.orientation.w = orient_w;
    return pose;
  }

  void print_pose(geometry_msgs::msg::PoseStamped pose)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "\npose:\n position:\n  x: " <<
        pose.pose.position.x << "\n  y: " <<
        pose.pose.position.y << "\n  z: " <<
        pose.pose.position.z << "\n orientation:\n  x: " <<
        pose.pose.orientation.x << "\n  y: " <<
        pose.pose.orientation.y << "\n  z: " <<
        pose.pose.orientation.z << "\n  w: " <<
        pose.pose.orientation.w << std::endl);
  }

  void print_tf(geometry_msgs::msg::TransformStamped tf)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "\ntf pose:\n position:\n  x: " <<
        tf.transform.translation.x << "\n  y: " <<
        tf.transform.translation.y << "\n  z: " <<
        tf.transform.translation.z << "\n orientation:\n  x: " <<
        tf.transform.rotation.x << "\n  y: " <<
        tf.transform.rotation.y << "\n  z: " <<
        tf.transform.rotation.z << "\n  w: " <<
        tf.transform.rotation.w << std::endl);
  }

  rclcpp::TimerBase::SharedPtr path_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}
