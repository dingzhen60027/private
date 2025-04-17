#ifndef IMAGELABELENLARGE_H
#define IMAGELABELENLARGE_H

#include <QWidget>
#include <QLabel>

#include "globalVar.h"
#include "globalFun.h"

class ImageLabelEnlarge : public QLabel
{
    Q_OBJECT
public:
    explicit ImageLabelEnlarge(QWidget *parent = nullptr);
    ~ImageLabelEnlarge();

    void colorAnalyse();
    void drawColorPoint(double h, double s, double i);

    QPoint   point[10];
    int      pointCount;
    QPoint*   drawPoints;
    int      drawPointCount;

    bool     m_switch;
    bool     m_draw_point;

protected:
    virtual void paintEvent(QPaintEvent *ev);
    virtual void mousePressEvent(QMouseEvent *ev);
    virtual void mouseReleaseEvent(QMouseEvent *ev);

signals:
    void updateColorImage(QPoint* drawpoints, int drawpointcount);
    void updateIdataImage();
public slots:
    void updateLabel(int num);  //此处的num只是为了符合connect槽函数的要求
};

#endif // IMAGELABELENLARGE_H
