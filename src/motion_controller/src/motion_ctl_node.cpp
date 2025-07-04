#include <cstdio>
#include <memory>
#include <cmath>

#include "motion_controller/motion_ctl_node.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

static constexpr double kSimRadius = 4.5;
static constexpr double kSimCenterX = 5.5;
static constexpr double kSimCenterY = 5.5;

static constexpr double kMotionControlDt = 0.01; // seconds
static constexpr double kLinearVelocity = 0.2;
static constexpr double kAngularVelocity = 0.5;

static constexpr double kPointThreshold = 0.2;
static constexpr double kMaxLinearSpeed = 0.5;
static constexpr double kMaxAngularSpeed = 0.5;

MotionControllerNode::MotionControllerNode()
    : Node("motion_controller_node")
{
  this->declare_parameter<bool>("use_teleport", true);
  m_use_teleport = this->get_parameter("use_teleport").as_bool();
  RCLCPP_INFO(this->get_logger(), "use_teleport = %d", m_use_teleport);

  m_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose", 10, std::bind(&MotionControllerNode::target_pose_callback, this, std::placeholders::_1));

  m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

  m_client = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
  // Wait for the service to be available
  if (!m_client->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Teleport service not available");
    return;
  }
}

void MotionControllerNode::command_turtle_bot_velocity()
{
  geometry_msgs::msg::Twist twist = calculate_target_velocity(m_target_position);
  publish_cmd_vel(twist);
  update_position_from_velocity(twist);
}

void MotionControllerNode::target_pose_callback(const geometry_msgs::msg::PoseStamped &pose)
{
  RCLCPP_INFO(this->get_logger(),
              "I heard...\n"
              "  frame_id: %s\n"
              "  position: [%.3f, %.3f, %.3f]",
              pose.header.frame_id.c_str(),
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z);

  if (m_use_teleport)
  {
    teleport_to_target_pose(pose.pose.position.x, pose.pose.position.y);
    m_current_position = pose.pose;
  }
  m_target_position = pose.pose;
}

void MotionControllerNode::publish_cmd_vel(const geometry_msgs::msg::Twist &twist) const
{
  RCLCPP_INFO(this->get_logger(),
              "Publishing a twist: \n"
              "Linear: x=%f, y=%f, z=%f\n"
              "Angular: x=%f, y=%f, z=%f\n",
              twist.linear.x, twist.linear.y, twist.linear.z,
              twist.angular.x, twist.angular.y, twist.angular.z);
  m_publisher->publish(twist);
}

void MotionControllerNode::teleport_to_target_pose(double x, double y)
{
  auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
  request->x = kSimCenterX + kSimRadius * x;
  request->y = kSimCenterY + kSimRadius * y;

  m_client->async_send_request(request);
}

void MotionControllerNode::update_position_from_velocity(geometry_msgs::msg::Twist twist)
{
  // Get current yaw from quaternion
  tf2::Quaternion q(
      m_current_position.orientation.x,
      m_current_position.orientation.y,
      m_current_position.orientation.z,
      m_current_position.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // Integrate position
  m_current_position.position.x += twist.linear.x * std::cos(yaw) * kMotionControlDt;
  m_current_position.position.y += twist.linear.x * std::sin(yaw) * kMotionControlDt;

  // Integrate orientation (yaw)
  yaw += twist.angular.z * kMotionControlDt;

  // Convert yaw back to quaternion
  tf2::Quaternion new_q;
  new_q.setRPY(0.0, 0.0, yaw);
  m_current_position.orientation.x = new_q.x();
  m_current_position.orientation.y = new_q.y();
  m_current_position.orientation.z = new_q.z();
  m_current_position.orientation.w = new_q.w();
}

geometry_msgs::msg::Twist MotionControllerNode::calculate_target_velocity(
    const geometry_msgs::msg::Pose &target) const
{
  geometry_msgs::msg::Twist cmd;

  double dx = target.position.x - m_current_position.position.x;
  double dy = target.position.y - m_current_position.position.y;

  double distance = std::hypot(dx, dy);
  double target_angle = std::atan2(dy, dx);

  // Extract current yaw from quaternion
  tf2::Quaternion q(
      m_current_position.orientation.x,
      m_current_position.orientation.y,
      m_current_position.orientation.z,
      m_current_position.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  double angle_error = target_angle - yaw;

  // Normalize angle
  while (angle_error > M_PI)
    angle_error -= 2 * M_PI;
  while (angle_error < -M_PI)
    angle_error += 2 * M_PI;

  // Only move forward if facing close enough
  if (std::abs(angle_error) < kPointThreshold)
  {
    RCLCPP_INFO(this->get_logger(),
                "Only move forward if facing close enough. \n");
    cmd.linear.x = std::min(kLinearVelocity * distance, kMaxLinearSpeed);
  }
  else
  {
    cmd.angular.z = std::clamp(kAngularVelocity * angle_error, -kMaxAngularSpeed, kMaxAngularSpeed);
  }

  return cmd;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotionControllerNode>();
  std::thread ros_spin_thread([&]()
                              { rclcpp::spin(node); });

  if (!node->get_use_teleport())
  {
    while (rclcpp::ok())
    {
      node->command_turtle_bot_velocity();
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(kMotionControlDt * 1000)));
    }
  }

  ros_spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
