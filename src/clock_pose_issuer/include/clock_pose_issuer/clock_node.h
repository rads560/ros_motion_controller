#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/* This class provides poses based on the minute hand of the current time. */

class ClockPoseIssuerNode : public rclcpp::Node
{
public:
    ClockPoseIssuerNode();

private:
    geometry_msgs::msg::PoseStamped convert_curr_time_to_pose();
    void publish_clock_pose();

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_publisher;
};
