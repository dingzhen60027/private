#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QButtonGroup>
//#include "rclcomm.h"
#include "rclcomm1.h"
#include "rclcomm2.h"
#include "rclcomm3.h"
#include "rclcomm4.h"
#include <iostream>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    QImage m_QImageForDisp;
    int centerfcalib;

    QButtonGroup* buttonGroup;

    QButtonGroup* buttonGroup_Attack;
    QButtonGroup* buttonGroup_HalfGround;
    QButtonGroup* buttonGroup_GameMode;
    QButtonGroup* buttonGroup_StartMode;
    QButtonGroup* buttonGroup_VarSet;

    void initDecisionButtonGroup();

private:
    Ui::MainWindow *ui;
    
    //rclcomm *commNode;
    rclcomm1 *commNode1;
    //rclcomm2 *commNode2;
    rclcomm3 *commNode3;
    rclcomm4 *commNode4;

signals:
    void Open_Camera_INFO();
    void Close_Camera_INFO();
    void Start_Acq_INFO();
    void Quit_Acq_INFO();
    void sendHSIcolor(int* HSIsender);

    void sendDecisionvar();

public slots:
    //void updateTopicInfo(QString);
    void updateTopicInfo_image_before(QImage);
    void updateTopicInfo_image_after(QImage);
    void updateSimuImage(QPixmap);
    //void on_myStopButton_clicked();
    void getRadioButtonID_A(int);
    void getRadioButtonID_H(int);
    void getRadioButtonID_G(int);
    void getRadioButtonID_S(int);
    void getRadioButtonID_V(int);
private slots:
    void on_Btn_Open_clicked();
    void on_Btn_Close_clicked();
    void on_Btn_Start_clicked();
    void on_Btn_Quit_clicked();
    void on_Btn_Load_clicked();
    void on_Btn_Save_clicked();

    void on_Btn_PointRecord_clicked();
    void on_Btn_PointCalib_clicked();
    void on_Btn_PointDelone_clicked();
    void on_Btn_PointDelall_clicked();

    void get_Color_Enlarge(QRect rect);
    void on_Btn_Sampling_clicked();
    void on_Btn_ColorTest_clicked();
    void on_Btn_RePoints_clicked();
    void on_Btn_HSIcomplete_clicked();
    void on_Btn_ToStart_clicked();
    void on_Btn_ToStop_clicked();
};
#endif // MAINWINDOW_H
