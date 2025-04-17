#ifndef IMAGELABELCONTROL_H
#define IMAGELABELCONTROL_H

#include <QObject>
#include <QLabel>

#include "globalVar.h"

class ImageLabelControl : public QLabel
{
    Q_OBJECT
public:
    explicit ImageLabelControl(QWidget *parent = nullptr);

    void boundset(Point* calipoint_);

    QRect getRect(QPoint bp, QPoint ep);

    int         x_pos;
    int         y_pos;

    QPoint      beginPoint;
    QPoint      endPoint;

    bool        m_isMousePress;
    bool        m_boundsetPress;
    bool        m_acquisition;

protected:
    virtual void mousePressEvent(QMouseEvent *ev);

    virtual void mouseMoveEvent(QMouseEvent *ev);

    virtual void mouseReleaseEvent(QMouseEvent *ev);

    virtual void paintEvent(QPaintEvent *ev);

signals:
    void pointDrew();
    void sendBoundsetPoint(Point* calipoint_);
    void sendRect(QRect rect);
public slots:
    
};

#endif // IMAGELABELCONTROL_H
