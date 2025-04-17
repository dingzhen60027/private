#include "ImageLabelControl.h"

#include <QPainter>
#include <QMouseEvent>

ImageLabelControl::ImageLabelControl(QWidget *parent) : QLabel(parent)
{
    x_pos=0;
    y_pos=0;
    m_isMousePress=false;
    m_boundsetPress=false;
    m_acquisition=false;
}

void ImageLabelControl::mousePressEvent(QMouseEvent *ev)
{
    update();
    x_pos=ev->pos().x();
    y_pos=ev->pos().y();
    beginPoint=ev->pos();
    m_isMousePress=true;
    return;
}

void ImageLabelControl::mouseMoveEvent(QMouseEvent *ev)
{
    if(m_isMousePress)
    {
        update();
        endPoint=ev->pos();
    }
    return;
}

void ImageLabelControl::mouseReleaseEvent(QMouseEvent *ev)
{
    endPoint=ev->pos();
    m_isMousePress=false;
    return;
}

void ImageLabelControl::paintEvent(QPaintEvent *ev)
{
    QLabel::paintEvent(ev);
    QPainter painter(this);
    QPen pen;
    pen.setWidth(2);

    if(m_isMousePress)//鼠标触发的绘图事件
    {
        if(!m_acquisition)//标定放大-停止采集时进入
        {
            //QRect re;
            //QPoint begin_point, end_point;
            //begin_point.x() = x_pos-50;
            //begin_point.y() = y_pos-50;
            //end_point.x() = begin_point.x() + 100;
            //end_point.y() = begin_point.y() + 100;
            //re=getRect(begin_point, end_point);
            pen.setColor(Qt::red);
            painter.setPen(pen);
            painter.drawPoint(x_pos,y_pos);
            emit pointDrew();
        }

        if(m_acquisition)//采色放大-开始采集时进入
        {
            QRect re;
            pen.setColor(Qt::lightGray);
            painter.setPen(pen);
            re=getRect(beginPoint,endPoint);
            painter.drawRect(re);
            emit sendRect(re);
        }
    }
}

void ImageLabelControl::boundset(Point* calipoint_)
{
    emit sendBoundsetPoint(calipoint_);

    //通过update触发绘图事件，在中心画出场地边框
    m_boundsetPress=true;

    return;
}

QRect ImageLabelControl::getRect(QPoint bp, QPoint ep)
{
    int x,y,width,height;
    width = qAbs(bp.x()-ep.x());
    height = qAbs(bp.y()-ep.y());
    x = bp.x()< ep.x()?bp.x():ep.x();
    y = bp.y()< ep.y()?bp.y():ep.y();

    QRect selectRect = QRect(x,y,width,height);


    return selectRect;
}
