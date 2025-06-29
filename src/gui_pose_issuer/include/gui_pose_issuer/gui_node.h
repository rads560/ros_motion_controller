#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

/* This class provides poses based on the minute hand from a GUI that the user clicks on. */

class GuiPoseIssuerNode : public rclcpp::Node
{
public:
    GuiPoseIssuerNode();
    void publish_gui_pose(double x, double y);
    void publish_abort_signal();

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_abort_publisher;
};
