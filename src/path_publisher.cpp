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
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(1).transient_local());
    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

    path_timer_callback();
    tf_timer_callback();

    path_timer_ = this->create_wall_timer(5s, std::bind(&PathPublisher::path_timer_callback, this));
    tf_timer_ = this->create_wall_timer(5s, std::bind(&PathPublisher::tf_timer_callback, this));
  }

private:
  void path_timer_callback()
  {
    auto message = nav_msgs::msg::Path();
    message.header.stamp = now();
    message.header.frame_id = "odom";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";

    // line along x-axis
    for (unsigned int i=0; i<3; i++) {
      pose.pose.position.x += 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      message.poses.push_back(pose);
    }

    // line along y-axis
    for (unsigned int i=0; i<3; i++) {
      pose.pose.position.y += 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.7071;
      pose.pose.orientation.w = 0.7071;
      message.poses.push_back(pose);
    }

    // line along z-axis
    for (unsigned int i=0; i<3; i++) {
      pose.pose.position.z += 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = -0.7071;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 0.7071;
      message.poses.push_back(pose);
    }

    // line between x and z axes
    for (unsigned int i=0; i<3; i++) {
      pose.pose.position.x += 1.0;
      pose.pose.position.z += 1.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.38268;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = -0.9239;
      message.poses.push_back(pose);
    }

    path_publisher_->publish(message);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing path" << std::endl);
  }

  void tf_timer_callback()
  {
    // create header
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "map";

    // create tf
    geometry_msgs::msg::Transform tf;
    auto rot = geometry_msgs::msg::Quaternion();
    rot.x = 0.0;
    rot.y = 0.0;
    rot.z = 1.0;
    rot.w = 1.0;
    auto trans = geometry_msgs::msg::Vector3();
    trans.x = 0.0;
    trans.y = 0.0;
    trans.z = 1.0;
    tf.rotation = rot;
    tf.translation = trans;

    // create stamped tf
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header = header;
    tf_stamped.child_frame_id = "odom";
    tf_stamped.transform = tf;

    // create vector of stamped tfs
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.push_back(tf_stamped);

    // create msg
    tf2_msgs::msg::TFMessage msg;
    msg.transforms = tfs;

    tf_publisher_->publish(msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing tf" << std::endl);
  }

  void print_pose(geometry_msgs::msg::PoseStamped pose)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "\npose:\n position:\n  x: " << 
      pose.pose.position.x << "\n  y: " <<
      pose.pose.position.y << "\n  z: " <<
      pose.pose.position.z << "\n orientation:\n  x: " <<
      pose.pose.orientation.x << "\n  y: " <<
      pose.pose.orientation.y << "\n  z: " <<
      pose.pose.orientation.z << "\n  w: " <<
      pose.pose.orientation.w << std::endl);    
  }

  void print_tf(geometry_msgs::msg::TransformStamped tf) {
    RCLCPP_INFO_STREAM(this->get_logger(), "\ntf pose:\n position:\n  x: " << 
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
