#include "HSIslider.h"
#include "ui_HSIslider.h"

#include "globalVar.h"
#include <QSlider>
HSIslider::HSIslider(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::HSIslider)
{
    ui->setupUi(this);

    //QSpinBox改变引起QSlider改变
    void (QSpinBox:: *spSignal)(int)=&QSpinBox::valueChanged;
    connect(ui->spinBox,spSignal,ui->horizontalScrollBar,&QSlider::setValue);

    //QSlider引起QSpinBox改变
    connect(ui->horizontalScrollBar,&QScrollBar::valueChanged,ui->spinBox,&QSpinBox::setValue);

    //数值改变时发送信号
    connect(ui->horizontalScrollBar,&QScrollBar::valueChanged,this,&HSIslider::sendChangedValued);
}

HSIslider::~HSIslider()
{
    delete ui;
}

void HSIslider::sendChangedValued(int value)
{
    emit valueIsChanged(value);

    return;
}

void HSIslider::setMaximumByNumIn(int num)
{
    ui->horizontalScrollBar->setMaximum(num);
    ui->spinBox->setMaximum(num);
    return;
}

void HSIslider::HToMax(int id)
{
    if(HSIThreshold[id][1]<=3600&&HSIThreshold[id][1]>=0)
    {
        ui->spinBox->setValue(HSIThreshold[id][1]);
        return;
    }
    ui->spinBox->setValue(3600);
    return;
}

void HSIslider::HToMini(int id)
{
    if(HSIThreshold[id][0]<=3600&&HSIThreshold[id][0]>=0)
    {
        ui->spinBox->setValue(HSIThreshold[id][0]);
        return;
    }
    ui->spinBox->setValue(0);
    return;
}

void HSIslider::SToMax(int id)
{
    if(HSIThreshold[id][3]<=100&&HSIThreshold[id][3]>=0)
    {
        ui->spinBox->setValue(HSIThreshold[id][3]);
        return;
    }
    ui->spinBox->setValue(100);
    return;
}

void HSIslider::SToMini(int id)
{
    if(HSIThreshold[id][2]<=100&&HSIThreshold[id][2]>=0)
    {
        ui->spinBox->setValue(HSIThreshold[id][2]);
        return;
    }
    ui->spinBox->setValue(0);
    return;
}

void HSIslider::IToMax(int id)
{
    if(HSIThreshold[id][5]<=255&&HSIThreshold[id][5]>=0)
    {
        ui->spinBox->setValue(HSIThreshold[id][5]);
        return;
    }
    ui->spinBox->setValue(255);
    return;
}

void HSIslider::IToMini(int id)
{
    if(HSIThreshold[id][4]<=255&&HSIThreshold[id][4]>=0)
    {
        ui->spinBox->setValue(HSIThreshold[id][4]);
        return;
    }
    ui->spinBox->setValue(0);
    return;
}
