#include "ImageLabelEnlarge.h"

#include <iostream>

#include <QMouseEvent>
#include <QPainter>
#include <QMessageBox>
#include <QDebug>
#include <QRgb>

ImageLabelEnlarge::ImageLabelEnlarge(QWidget *parent) : QLabel(parent)
{
    pointCount=0;
    drawPointCount=0;
    m_switch=false;
    m_draw_point=false;

    drawPoints = new QPoint[COLORENLARGE_WIDTH*COLORENLARGE_HEIGHT];
    if(drawPoints == NULL)
    {
        qDebug()<<"drawPoints无法动态申请内存!!";
    }
}
ImageLabelEnlarge::~ImageLabelEnlarge()
{
    if(drawPoints != NULL)
    {
        delete[] drawPoints;
        qDebug()<<"已经清除drawPoints所占内存!";
    }
}

void ImageLabelEnlarge::mousePressEvent(QMouseEvent *ev)
{
    if (pointCount < 10)
    {
        point[pointCount]=ev->pos();
        m_draw_point = true;
        update();
    }
    return;
}

void ImageLabelEnlarge::mouseReleaseEvent(QMouseEvent *ev)
{
    if (pointCount < 10)
    {
        pointCount++;
        std::cout << "选中第" << pointCount << "个图像坐标 " << "x:" << ev->pos().x() << " " << "y:" << ev->pos().y() << std::endl;
    }
    else
    {
        QMessageBox msg;
        msg.setText("最多选择10个范围点!!当前已经选择10个!");
        msg.exec();
    }
    return;
}

void ImageLabelEnlarge::paintEvent(QPaintEvent *ev)
{
    QLabel::paintEvent(ev);

    QPainter painter(this);
    QPen pen;
    if(m_switch)
    {
        painter.eraseRect(QRect(0,0,COLORENLARGE_WIDTH,COLORENLARGE_HEIGHT));
        m_switch=false;
        return;
    }
    if(m_draw_point)
    {
        pen.setColor(Qt::red);
        pen.setWidth(3);
        painter.setPen(pen);
        //QVector<QPoint> vec;
        //painter.drawPoints(&vec[0],vec.size());
        painter.drawPoints(point,pointCount+1);
        m_draw_point=false;
    }

    return;
}

void ImageLabelEnlarge::updateLabel(int num)
{
    std::cout << "updateNum:" << num << std::endl;
    m_switch=true;
    pointCount=0;
    update();
    return;
}

void ImageLabelEnlarge::colorAnalyse()
{
    int R,G,B;
    double S,H;
    int I;

    if(pointCount<3)
    {
        QMessageBox msg;
        msg.setText("最少选择3个范围点!!");
        msg.exec();
        return;
    }

    if(drawPointCount!=0)
    {
        drawPointCount=0;
    }

    QPixmap pix = QWidget::grab(QRect(QPoint(0,0),QSize(COLORENLARGE_WIDTH,COLORENLARGE_HEIGHT)));
    QImage img = pix.toImage();
    QVector<QPoint> pointVector;

    for(int i=0;i<pointCount;i++)
    {
        pointVector.append(point[i]);
    }
    for(int i=0; i<255; i++)
    {
        I_y[i]=0;
    }

    QPolygon polygon(pointVector);
    for(int i=0;i<COLORENLARGE_WIDTH;i++)
    {
        for(int j=0;j<COLORENLARGE_HEIGHT;j++)
        {
            if(polygon.containsPoint(QPoint(i,j),Qt::OddEvenFill))
            {
                QColor color = img.pixel(i,j);
                R = color.red();
                G = color.green();
                B = color.blue();
                H = HLUT[R][G][B] * Pi / 180.0;
                S = 1 - 3.0 * countMin(R, G, B, 3) / (double(R)+ double(G) + double(B));
                I = (int)(R + G + B) / 3;

                I_y[I]++;
                drawColorPoint(H,S,I);
            }
        }
    }
    emit updateColorImage(drawPoints, drawPointCount);
    emit updateIdataImage();

    return;
}

void ImageLabelEnlarge::drawColorPoint(double h, double s, double i)
{
    double H,S,I;
    H = h;
    S = s;
    I = i;
    QPoint colorPoint = QPoint(int(COLORCIR * S * cos(H) + COLORCIR),int(COLORCIR * S * sin(H) + COLORCIR));
    qDebug() << "颜色分析结果  " << "H:" << H << " S:" << S << " I:" << I;

    //QImage   image(COLORDATA_WIDTH,COLORDATA_HEIGHT,QImage::Format_ARGB32);
    //QPainter painter(&image);
    //QPen     pen;
    //pen.setColor(Qt::black);
    //pen.setWidth(2);
    //painter.drawPoint(colorPoint);

    drawPoints[drawPointCount] = colorPoint;
    drawPointCount++;

    return;
}
