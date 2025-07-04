#include <chrono>
#include <functional>
#include <ctime>

#include "clock_pose_issuer/clock_node.h"

/* This class provides poses based on the minute hand of the current time. */

ClockPoseIssuerNode::ClockPoseIssuerNode()
    : Node("clock_pose_issuer_node")
{
  m_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("clock_pose", 10);
  m_timer = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&ClockPoseIssuerNode::publish_clock_pose, this));
}

geometry_msgs::msg::PoseStamped ClockPoseIssuerNode::convert_curr_time_to_pose()
{
  std::time_t now = std::time(nullptr);
  std::tm *local_tm = std::localtime(&now);
  double mins = local_tm->tm_min + (local_tm->tm_sec / 60.0);
  RCLCPP_INFO(this->get_logger(), "Minutes: %f", mins);

  // total of 60 mins in 360 degrees or 2pi radians
  double theta = (mins / 60.0) * 2 * M_PI;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = "global";

  pose.pose.position.x = std::sin(theta);
  pose.pose.position.y = std::cos(theta);

  return pose;
}

void ClockPoseIssuerNode::publish_clock_pose()
{
  geometry_msgs::msg::PoseStamped pose = convert_curr_time_to_pose();
  RCLCPP_INFO(this->get_logger(),
              "Publishing PoseStamped:\n"
              "  frame_id: %s\n"
              "  position: [%.3f, %.3f, %.3f]",
              pose.header.frame_id.c_str(),
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z);
  m_publisher->publish(pose);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockPoseIssuerNode>());
  rclcpp::shutdown();
  return 0;
}
