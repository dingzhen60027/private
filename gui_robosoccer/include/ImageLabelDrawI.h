#ifndef IMAGELABELDRAWI_H
#define IMAGELABELDRAWI_H

#include <QWidget>
#include <QLabel>

class ImageLabelDrawI : public QLabel
{
    Q_OBJECT
public:
    explicit ImageLabelDrawI(QWidget *parent = nullptr);

    void getILOWNum(int num4);
    void getIHIGHNum(int num5);


    int  HLUT_FN;
    int  I_LOW,I_HIGH;

    bool m_switch;

protected:
    virtual void paintEvent(QPaintEvent *ev);

signals:

public slots:
    void getRadioButtonID(int id);
    void getIimage();
};

#endif // IMAGELABELDRAWI_H
