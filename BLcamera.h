#pragma once

#include <QtWidgets/QWidget>
#include "ui_BLcamera.h"
#include <QIcon>
#include <QMouseEvent>
#include <QStatusBar>
#include <QMessageBox>
#include <QThread>
#include "MvCamera.h"
#include "StructDefine.h"
#include "yolov5.h"

class BLcamera : public QWidget
{
    Q_OBJECT

public:
    CAMERA_STATUS m_cameraStatue;               //相机状态
    MYCAMERA m_myCamera[MAX_DEVICE_NUM];        //设备相关
    MV_CC_DEVICE_INFO_LIST m_stDevList;         //设备信息列表
public:
    GrabThread* grabThread_LeftCamera = NULL;  //左相机线程对象
    GrabThread* grabThread_RightCamera = NULL; //右相机线程对象
    DetectThread* detect = NULL;                     //测距线程对象
public:
    BLcamera(QWidget *parent = Q_NULLPTR);
    ~BLcamera();
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void InitBtnConnect();//实现按钮信号槽绑定

    void InitCamera();//初始化相机,按钮初始化响应
    void RectifyMapping();// 极线匹配映射表
    void ClickTriggerModeRadio();//按钮触发模式响应
    void ClickConModeRadio();    //按钮连续模式响应
    void ClickStartGrabbing();   //按钮开始采集
    void ClickMeasure();         //按钮开始测距
    void ClickSendData();       //按钮发送
    int  ClickClose();
    int ClickSave();            //保存图片
    int ClickSave2BMP();
    int ClickSave2PNG();
    int ClickSave2TIFF();
    int ClickSave2JPG();

    /*******************************************************/
    int EnumDevices();                         //枚举设备
    void ShowErrorMsg(QString,int);             //显示错误信息
    int OpenDevices();                          //打开设备
    int DownLoadIniFile();                     //加载ini文件
    int SetParameters();                       //设置相机参数（曝光时间、增益、帧率）
    int SetExposureTime();                     //设置曝光时间
    int SetGain();                             //设置增益
    int SetFrameRate();                        //设置帧率
    int StartGrabbing();                       //开始采集  
    void EnableBtn();                          //按钮使能
    int SaveImage();
    void UpdateParams();                       //更新参数
    void Save2Ini();                           //导出至ini文件
    void SetCalibration();                    //设置标定参数
    
    //slots
    void DisplayOnRightSlot(cv::Mat* img);
    void DisplayPositionSlot(double,double,double,double);
public:
    Ui::BLcameraClass ui;
    QPoint p;
signals:
    void startGrabbing(BLcamera*, int);
};


