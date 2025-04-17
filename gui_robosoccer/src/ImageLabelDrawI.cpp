#include "ImageLabelDrawI.h"

#include <QPainter>
#include "globalVar.h"

ImageLabelDrawI::ImageLabelDrawI(QWidget *parent) : QLabel(parent)
{
    HLUT_FN=0;
    I_LOW=0;
    I_HIGH=0;
    
    m_switch=false;
}

void ImageLabelDrawI::paintEvent(QPaintEvent *ev)
{
    QLabel::paintEvent(ev);
    QPainter painter(this);

    if(m_switch)
    {
        painter.eraseRect(QRect(0,0,SHOWI_WIDTH,SHOWI_HEIGHT));
        m_switch=false;
        return;
    }

    QPen pen;
    pen.setColor(Qt::green);
    pen.setWidth(2);
    painter.setPen(pen);

    QPoint start1(I_LOW*1.2,60);
    QPoint end1(I_LOW*1.2,20);
    QPoint start2(I_HIGH*1.2,60);
    QPoint end2(I_HIGH*1.2,20);
    painter.drawLine(start1,end1);
    painter.drawLine(start2,end2);

    QPen pen1;
    pen1.setColor(Qt::blue);
    pen1.setWidth(2);
    painter.setPen(pen1);
    for (int j = 0;j < 255;j++)
    {
        if (I_y[j] == 0)
        {
            continue;
        }
        else
        {
          if(I_y[j] > 60)
              I_y[j] = 60;
          painter.drawLine(j*1.2,60,j*1.2,60-I_y[j]);      //绘制亮度的高度

        }
     }

}

void ImageLabelDrawI::getIimage()
{
    update();
    return;
}

void ImageLabelDrawI::getRadioButtonID(int id)
{
    HLUT_FN=id;

    m_switch=true;
    update();
    return;
}

void ImageLabelDrawI::getILOWNum(int num4)
{
    if(HLUT_FN>=0&&HLUT_FN<8)
    {
        HSIThreshold[HLUT_FN][4] = num4;
        I_LOW = HSIThreshold[HLUT_FN][4];
    }
    update();
    return;
}

void ImageLabelDrawI::getIHIGHNum(int num5)
{
    if(HLUT_FN>=0&&HLUT_FN<8)
    {
        HSIThreshold[HLUT_FN][5] = num5;
        I_HIGH = HSIThreshold[HLUT_FN][5];
    }
    update();
    return;
}
