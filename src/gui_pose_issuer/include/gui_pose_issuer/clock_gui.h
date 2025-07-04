#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QKeyEvent>

#include "gui_pose_issuer/gui_node.h"

class ClockWidget : public QWidget
{
public:
    ClockWidget(const std::shared_ptr<GuiPoseIssuerNode> &gui_node, QWidget *parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    void set_minute(int minute);

    std::shared_ptr<GuiPoseIssuerNode> m_gui_node;
    double m_minute_angle = 0.0;
};
