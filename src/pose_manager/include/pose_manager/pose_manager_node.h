#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

class PoseManagerNode : public rclcpp::Node
{
public:
    PoseManagerNode();

private:
    void create_timer();
    void reset_timer();
    void stop_timer();
    void clock_pose_callback(const geometry_msgs::msg::PoseStamped &pose) const;
    void gui_pose_callback(const geometry_msgs::msg::PoseStamped &pose);
    void abort_pose_callback(const std_msgs::msg::Empty);
    void publish_target_pose(const geometry_msgs::msg::PoseStamped &pose) const;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_clock_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_gui_subscription;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_abort_subscription;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_publisher;

    rclcpp::TimerBase::SharedPtr m_inactivity_timer;
    bool m_use_clock_pose = true;
};
