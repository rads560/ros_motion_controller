#include <QApplication>
#include <QDebug>
#include <cmath>

#include "gui_pose_issuer/clock_gui.h"

ClockWidget::ClockWidget(const std::shared_ptr<GuiPoseIssuerNode> &gui_node, QWidget *parent) : QWidget(parent), m_gui_node(gui_node)
{
    setFixedSize(400, 400);
}

void ClockWidget::set_minute(int minute)
{
    // Minute hand moves 6 degrees per minute
    m_minute_angle = minute * ((M_PI * 2) / 60);
    update();
}

void ClockWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QPoint center = rect().center();
    int radius = std::min(width(), height()) / 2 - 20;

    // Draw clock circle
    painter.drawEllipse(center, radius, radius);

    // Draw clock ticks
    for (int i = 0; i < 12; ++i)
    {
        double angle = i * M_PI / 6;
        int x1 = center.x() + std::cos(angle) * (radius - 10);
        int y1 = center.y() - std::sin(angle) * (radius - 10);
        int x2 = center.x() + std::cos(angle) * radius;
        int y2 = center.y() - std::sin(angle) * radius;
        painter.drawLine(QPoint(x1, y1), QPoint(x2, y2));
    }

    // Draw minute hand
    painter.setPen(QPen(Qt::black, 4, Qt::SolidLine, Qt::RoundCap));
    QPoint minute_end(center.x() + std::cos(m_minute_angle) * (radius * 0.8),
                      center.y() - std::sin(m_minute_angle) * (radius * 0.8));
    painter.drawLine(center, minute_end);
}

void ClockWidget::mousePressEvent(QMouseEvent *event)
{
    QPointF clickPos = event->pos();
    QPointF center = rect().center();

    double dx = clickPos.x() - center.x();
    double dy = center.y() - clickPos.y(); // Qt Y axis is inverted
    double r = std::sqrt(dx * dx + dy * dy);
    double radius = std::min(width(), height()) / 2 - 20;

    double theta = 0.0;
    if (std::abs(r - radius) < 30)
    {
        theta = std::atan2(dy, dx); // angle in radians
        if (theta < 0)
        {
            theta += 2 * M_PI; // normalize to 0 to 2 pi
        }
        qDebug("Clicked angle: %.2f radians (%.1f degrees)", theta, theta * 180.0 / M_PI);

        int minute = static_cast<int>(std::round(theta / (2 * M_PI) * 60)) % 60;
        set_minute(minute);
        qDebug("Clicked minute: %d", minute);
        RCLCPP_INFO(m_gui_node->get_logger(), "Clicked minute: %d", minute);

        dx = std::cos(theta);
        dy = std::sin(theta);

        m_gui_node->publish_gui_pose(dx, dy);
    }
}

void ClockWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Space)
    {
        qDebug("Space bar pressed!");
        m_gui_node->publish_abort_signal();
    }
    else
    {
        QWidget::keyPressEvent(event); // Pass to base class for other keys
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto gui_node = std::make_shared<GuiPoseIssuerNode>();

    std::thread ros_spin_thread([&]()
                                { rclcpp::spin(gui_node); });

    ClockWidget widget(gui_node);
    widget.setWindowTitle("Clock Pose GUI");
    widget.show();
    int result = app.exec();

    rclcpp::shutdown();
    ros_spin_thread.join();

    return result;
}
