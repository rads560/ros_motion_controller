#include "gui_pose_issuer/gui_node.h"

GuiPoseIssuerNode::GuiPoseIssuerNode()
    : Node("gui_pose_issuer_node")
{
  m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("gui_pose", 10);
  m_abort_publisher = this->create_publisher<std_msgs::msg::Empty>("abort_pose", 10);
}

void GuiPoseIssuerNode::publish_gui_pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = "global";
  pose.pose.position.x = x;
  pose.pose.position.y = y;

  RCLCPP_INFO(this->get_logger(),
              "Publishing PoseStamped:\n"
              "  frame_id: %s\n"
              "  position: [%.3f, %.3f, %.3f]",
              pose.header.frame_id.c_str(),
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z);
  m_pose_publisher->publish(pose);
}

void GuiPoseIssuerNode::publish_abort_signal()
{
  RCLCPP_INFO(this->get_logger(),
              "Publishing abort signal for last gui pose.");
  std_msgs::msg::Empty msg;
  m_abort_publisher->publish(msg);
}
