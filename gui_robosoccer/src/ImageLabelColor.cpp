#include "ImageLabelColor.h"

#include <QPainter>
#include <QDebug>
#include <QImage>

#include "globalVar.h"
#include "globalFun.h"

ImageLabelColor::ImageLabelColor(QWidget *parent) : QLabel(parent)
{    
    H=0;
    S=0;
    I=0;
    
    HLUT_FN=0;
    
    H_LOW=0;
    H_HIGH=0;
    S_LOW=0;
    S_HIGH=0;
    I_LOW=0;
    I_HIGH=0;
    
    m_drawPoints=false;
    m_switch=false;

    pointsImage.load(":/res/images/HSICir.bmp");
}

void ImageLabelColor::paintEvent(QPaintEvent *ev)
{
    QLabel::paintEvent(ev);
    QPainter painter(this);

    if(m_switch)
    {
        painter.eraseRect(0,0,COLORDATA_WIDTH,COLORDATA_HEIGHT);
        m_switch = false;
        painter.drawPixmap(QRect(0,0,COLORDATA_WIDTH,COLORDATA_HEIGHT),QPixmap(":/res/images/HSICir.bmp"));
        m_drawPoints = false;
        return;
    }

    if (m_drawPoints)
    {
        QPen pen_analyse;
        pen_analyse.setColor(Qt::black);
        pen_analyse.setWidth(2);
        painter.setPen(pen_analyse);
        for (int i=0; i<drawPointCount; i++)
        {
            painter.drawPoint(drawPoints[i]);
        }
        //m_drawPoints = false;
        //painter.drawImage(0,0,pointsImage);
    }

    QPen pen;
    pen.setColor(Qt::white);
    pen.setWidth(2);
    painter.setPen(pen);

    QRect rect1(COLORCIR-S_LOW,COLORCIR-S_LOW,2*S_LOW,2*S_LOW);
    QRect rect2(COLORCIR-S_HIGH,COLORCIR-S_HIGH,2*S_HIGH,2*S_HIGH);

    QPoint start1(int(COLORCIR + S_LOW * cos(H_HIGH * Pi / 1800.0)),int(COLORCIR + S_LOW * sin(H_HIGH * Pi / 1800.0)));
    QPoint end1(int(COLORCIR + S_LOW * cos(H_LOW * Pi / 1800.0)),int(COLORCIR + S_LOW * sin(H_LOW * Pi / 1800.0)));
    QPoint start2(int(COLORCIR + S_HIGH* cos(H_HIGH * Pi / 1800.0)),int(COLORCIR + S_HIGH * sin(H_HIGH* Pi /1800.0)));
    QPoint end2(int(COLORCIR + S_HIGH *cos(H_LOW * Pi / 1800.0)),int(COLORCIR + S_HIGH * sin(H_LOW * Pi /1800.0 )));

    painter.drawArc(rect1,(360-H_HIGH/10)*16,(H_HIGH - H_LOW)/10*16);
    painter.drawArc(rect2,(360-H_HIGH/10)*16,(H_HIGH - H_LOW)/10*16);

    painter.drawLine(start1,start2);
    painter.drawLine(end1,end2);

    update();
}

void ImageLabelColor::getColorImage(QPoint* drawpoints, int drawpointcount)
{
    //pointsImage = temp;
    drawPoints = drawpoints;
    drawPointCount = drawpointcount;

    m_drawPoints = true;
    update();
    return;
}

void ImageLabelColor::getRadioButtonID(int id)
{
    HLUT_FN = id;

    m_switch=true;
    update();
    return;
}

void ImageLabelColor::getHLOWNum(int num)
{
    if(HLUT_FN>=0&&HLUT_FN<8)
    {
        HSIThreshold[HLUT_FN][0] = num;
        H_LOW = HSIThreshold[HLUT_FN][0];
    }
    update();
    return;
}

void ImageLabelColor::getHHIGHNUm(int num1)
{
    if(HLUT_FN>=0&&HLUT_FN<8)
    {
        HSIThreshold[HLUT_FN][1] = num1;
        H_HIGH = HSIThreshold[HLUT_FN][1];
    }
    update();
    return;
}

void ImageLabelColor::getSLOWNum(int num2)
{
    if(HLUT_FN>=0&&HLUT_FN<8)
    {
        HSIThreshold[HLUT_FN][2] = num2;
        S_LOW = HSIThreshold[HLUT_FN][2];
    }
    update();
    return;
}

void ImageLabelColor::getSHIGHNum(int num3)
{
    if(HLUT_FN>=0&&HLUT_FN<8)
    {
        HSIThreshold[HLUT_FN][3] = num3;
        S_HIGH = HSIThreshold[HLUT_FN][3];
    }
    update();
    return;
}

void ImageLabelColor::getILOWNum(int num4)
{
    if(HLUT_FN>=0&&HLUT_FN<8)
        HSIThreshold[HLUT_FN][4] = num4;
    return;
}

void ImageLabelColor::getIHIGHNum(int num5)
{
    if(HLUT_FN>=0&&HLUT_FN<8)
        HSIThreshold[HLUT_FN][5] = num5;
    return;
}

QImage ImageLabelColor::colorTest(QImage imageIN)
{
    int img_width = imageIN.width();
    int img_height = imageIN.height();
    QImage img(img_width,img_height,QImage::Format_RGB888);
    int R,G,B;
    QColor COLOR;
    QRgb rgb;
    if(HSIThreshold[HLUT_FN][1]>HSIThreshold[HLUT_FN][0])
    {
        for(int i=0;i<img_width;i++)
        {
            for(int j=0;j<img_height;j++)
            {
                rgb = imageIN.pixel(i,j);
                COLOR = imageIN.pixel(i,j);
                R = COLOR.red();
                B = COLOR.blue();
                G = COLOR.green();
                H = 10 * HLUT[R][G][B];
                S = int(COLORCIR * (1 - 3.0 * countMin(R, G, B, 3) / (double(R) + double(G) + double(B))));
                I = int((R + G + B) / 3);
                if( H>=HSIThreshold[HLUT_FN][0]&&
                    H<=HSIThreshold[HLUT_FN][1]&&
                    S>=HSIThreshold[HLUT_FN][2]&&
                    S<=HSIThreshold[HLUT_FN][3]&&
                    I>=HSIThreshold[HLUT_FN][4]&&
                    I<=HSIThreshold[HLUT_FN][5] )
                {
                    img.setPixel(i,j,rgb);
                }
                else
                {
                    rgb = qRgb(255,255,255);
                    img.setPixel(i,j,rgb);
                }
            }
        }
    }
    else if(HSIThreshold[HLUT_FN][1]<HSIThreshold[HLUT_FN][0])
    {
        for(int i=0;i<img_width;i++)
        {    
            for(int j=0;j<img_height;j++)
            {
                rgb = imageIN.pixel(i,j);
                COLOR = imageIN.pixel(i,j);
                R = COLOR.red();
                B = COLOR.blue();
                G = COLOR.green();
                H = 10 * HLUT[R][G][B];
                S = int(COLORCIR * (1 - 3.0 * countMin(R, G, B, 3) / (double(R) + double(G) + double(B))));
                I = int((R + G + B) / 3);
                if( H>=HSIThreshold[HLUT_FN][0]||
                    (H<=HSIThreshold[HLUT_FN][1]&&
                     S>=HSIThreshold[HLUT_FN][2]&&
                     S<=HSIThreshold[HLUT_FN][3]&&
                     I>=HSIThreshold[HLUT_FN][4]&&
                     I<=HSIThreshold[HLUT_FN][5]) )
                {
                    img.setPixel(i,j,rgb);
                }
                else
                {
                    rgb = qRgb(255,255,255);
                    img.setPixel(i,j,rgb);
                }
            }
        }
    }

    return img;
}
