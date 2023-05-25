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
    CAMERA_STATUS m_cameraStatue;               //���״̬
    MYCAMERA m_myCamera[MAX_DEVICE_NUM];        //�豸���
    MV_CC_DEVICE_INFO_LIST m_stDevList;         //�豸��Ϣ�б�
public:
    GrabThread* grabThread_LeftCamera = NULL;  //������̶߳���
    GrabThread* grabThread_RightCamera = NULL; //������̶߳���
    DetectThread* detect = NULL;                     //����̶߳���
public:
    BLcamera(QWidget *parent = Q_NULLPTR);
    ~BLcamera();
    void mousePressEvent(QMouseEvent* e);
    void mouseMoveEvent(QMouseEvent* e);
    void InitBtnConnect();//ʵ�ְ�ť�źŲ۰�

    void InitCamera();//��ʼ�����,��ť��ʼ����Ӧ
    void RectifyMapping();// ����ƥ��ӳ���
    void ClickTriggerModeRadio();//��ť����ģʽ��Ӧ
    void ClickConModeRadio();    //��ť����ģʽ��Ӧ
    void ClickStartGrabbing();   //��ť��ʼ�ɼ�
    void ClickMeasure();         //��ť��ʼ���
    void ClickSendData();       //��ť����
    int  ClickClose();
    int ClickSave();            //����ͼƬ
    int ClickSave2BMP();
    int ClickSave2PNG();
    int ClickSave2TIFF();
    int ClickSave2JPG();

    /*******************************************************/
    int EnumDevices();                         //ö���豸
    void ShowErrorMsg(QString,int);             //��ʾ������Ϣ
    int OpenDevices();                          //���豸
    int DownLoadIniFile();                     //����ini�ļ�
    int SetParameters();                       //��������������ع�ʱ�䡢���桢֡�ʣ�
    int SetExposureTime();                     //�����ع�ʱ��
    int SetGain();                             //��������
    int SetFrameRate();                        //����֡��
    int StartGrabbing();                       //��ʼ�ɼ�  
    void EnableBtn();                          //��ťʹ��
    int SaveImage();
    void UpdateParams();                       //���²���
    void Save2Ini();                           //������ini�ļ�
    void SetCalibration();                    //���ñ궨����
    
    //slots
    void DisplayOnRightSlot(cv::Mat* img);
    void DisplayPositionSlot(double,double,double,double);
public:
    Ui::BLcameraClass ui;
    QPoint p;
signals:
    void startGrabbing(BLcamera*, int);
};


