#include <chrono>

#include "pose_manager/pose_manager_node.h"

/* The pose manager takes in poses from both the clock and gui nodes and decides what to send to the motion controller. */

PoseManagerNode::PoseManagerNode()
    : Node("pose_manager_node")
{
  m_clock_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "clock_pose", 10, std::bind(&PoseManagerNode::clock_pose_callback, this, std::placeholders::_1));
  m_gui_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "gui_pose", 10, std::bind(&PoseManagerNode::gui_pose_callback, this, std::placeholders::_1));
  m_abort_subscription = this->create_subscription<std_msgs::msg::Empty>(
      "abort_pose", 10, std::bind(&PoseManagerNode::abort_pose_callback, this, std::placeholders::_1));

  m_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
  create_timer();
}

void PoseManagerNode::create_timer()
{
  m_inactivity_timer = this->create_wall_timer(
      std::chrono::seconds(30),
      [this]()
      {
        RCLCPP_INFO(this->get_logger(), "30s inactivity timer expired, switching to clock following mode.");
        m_use_clock_pose = true;
      });
  m_inactivity_timer->cancel(); // Don't run until we explicitly start it
}

void PoseManagerNode::reset_timer()
{
  if (m_inactivity_timer)
  {
    m_inactivity_timer->cancel();
    m_inactivity_timer->reset();
  }
  RCLCPP_INFO(this->get_logger(), "Timer reset");
}

void PoseManagerNode::stop_timer()
{
  if (m_inactivity_timer)
  {
    m_inactivity_timer->cancel();
  }
  RCLCPP_INFO(this->get_logger(), "Timer stopped");
}

void PoseManagerNode::clock_pose_callback(const geometry_msgs::msg::PoseStamped &pose) const
{
  RCLCPP_INFO(this->get_logger(),
              "I heard...\n"
              "  frame_id: %s\n"
              "  position: [%.3f, %.3f, %.3f]",
              pose.header.frame_id.c_str(),
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z);
  if (m_use_clock_pose)
  {
    publish_target_pose(pose);
  }
}

void PoseManagerNode::gui_pose_callback(const geometry_msgs::msg::PoseStamped &pose)
{
  reset_timer();
  m_use_clock_pose = false;
  RCLCPP_INFO(this->get_logger(),
              "I heard...\n"
              "  frame_id: %s\n"
              "  position: [%.3f, %.3f, %.3f]",
              pose.header.frame_id.c_str(),
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z);
  publish_target_pose(pose);
}

void PoseManagerNode::abort_pose_callback(const std_msgs::msg::Empty)
{
  stop_timer();
  m_use_clock_pose = true;
}

void PoseManagerNode::publish_target_pose(const geometry_msgs::msg::PoseStamped &pose) const
{
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
  rclcpp::spin(std::make_shared<PoseManagerNode>());
  rclcpp::shutdown();
  return 0;
}
