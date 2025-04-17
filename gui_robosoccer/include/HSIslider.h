#ifndef HSISLIDER_H
#define HSISLIDER_H

#include <QWidget>

namespace Ui {
class HSIslider;
}

class HSIslider : public QWidget
{
    Q_OBJECT

public:
    explicit HSIslider(QWidget *parent = 0);
    ~HSIslider();


    void setMaximumByNumIn(int num);                    //设置Slider和spinbox的最大值

    void sendChangedValued(int value);                  //发送Slider和spinbox改变的值的函数

signals:
    void valueIsChanged(int value1);                    //Slider和spinbox值改变的信号

private:
    Ui::HSIslider *ui;

public slots:
    void HToMax(int id);                                //如果H有范围内的值，则设置为该值，否则将H设置为最大值，以下同理
    void HToMini(int id);
    void SToMax(int id);
    void SToMini(int id);
    void IToMax(int id);
    void IToMini(int id);
};


#endif // HSISLIDER_H
