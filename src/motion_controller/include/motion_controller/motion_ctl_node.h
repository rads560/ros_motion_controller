#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"

/* A motion controller that allows motion to be viewed with turtle sim. Supports teleporting mode and an untuned motion controller. */

class MotionControllerNode : public rclcpp::Node
{
public:
    MotionControllerNode();

    // Update turtle bot velocity based on current and target positions
    void command_turtle_bot_velocity();

    // When true, turtle bot teleports to target position directly
    // When false, turtle bot will use velocity commands to move to target position
    bool get_use_teleport()
    {
        return m_use_teleport;
    }

private:
    void target_pose_callback(const geometry_msgs::msg::PoseStamped &pose);
    void publish_cmd_vel(const geometry_msgs::msg::Twist &twist) const;
    void teleport_to_target_pose(double x, double y);
    void update_position_from_velocity(geometry_msgs::msg::Twist twist);
    geometry_msgs::msg::Twist calculate_target_velocity(
        const geometry_msgs::msg::Pose &target) const;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr m_client;

    geometry_msgs::msg::Pose m_current_position;
    geometry_msgs::msg::Pose m_target_position;
    bool m_use_teleport;
};
