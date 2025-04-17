#ifndef IMAGELABELCOLOR_H
#define IMAGELABELCOLOR_H

#include <QWidget>
#include <QLabel>

class ImageLabelColor : public QLabel
{
    Q_OBJECT
public:
    explicit ImageLabelColor(QWidget *parent = nullptr);

    void getHLOWNum(int num);
    void getHHIGHNUm(int num1);
    void getSLOWNum(int num2);
    void getSHIGHNum(int num3);
    void getILOWNum(int num4);
    void getIHIGHNum(int num5);

    QImage colorTest(QImage imageIN);

    int H,S,I;

    int HLUT_FN;

    int H_LOW,H_HIGH,S_LOW,S_HIGH,I_LOW,I_HIGH;

    QPoint* drawPoints;
    int drawPointCount;

    QVector<QPoint> points;
    QImage pointsImage;

    bool m_drawPoints;
    bool m_switch;

protected:
    virtual void paintEvent(QPaintEvent *ev);

signals:

public slots:
    void getRadioButtonID(int id);
    void getColorImage(QPoint* drawpoints, int drawpointcount);
};

#endif // IMAGELABELCOLOR_H
