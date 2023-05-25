#include "BLcamera.h"
#include <QDebug>


#pragma execution_character_set("utf-8")

BLcamera::BLcamera(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);  
    m_cameraStatue.bOpenDevice = false;
    m_cameraStatue.bStartGrabbing = false;
    m_cameraStatue.bMeasure = false;
    m_myCamera[0].myImage = new cv::Mat();
    m_myCamera[1].myImage = new cv::Mat();
    m_myCamera[0].pcMyCamera = NULL;
    m_myCamera[1].pcMyCamera = NULL;
    m_cameraStatue.nSingleOrDouble = 2;
    m_cameraStatue.stereoParams.IntrinsicMatrixCam1.create(cv::Size(3,3), CV_64FC1);
    m_cameraStatue.stereoParams.IntrinsicMatrixCam2.create(cv::Size(3, 3), CV_64FC1);
    m_cameraStatue.stereoParams.RotationMatrixOfCam2.create(cv::Size(3, 3), CV_64FC1);
    m_cameraStatue.stereoParams.TranslationMatrixOfCam2.create(cv::Size(1, 3), CV_64FC1);
    m_cameraStatue.stereoParams.DistortionMatrixCam1.create(cv::Size(1, 4), CV_64FC1);
    m_cameraStatue.stereoParams.DistortionMatrixCam2.create(cv::Size(1, 4), CV_64FC1);
    m_cameraStatue.stereoParams.RoiCam1.create(cv::Size(1, 4), CV_64FC1);
    m_cameraStatue.stereoParams.RoiCam2.create(cv::Size(1, 4), CV_64FC1);
    setWindowIcon(QIcon(":/image/windowIcon.png"));
    setWindowFlags(Qt::FramelessWindowHint | windowFlags());
    setFixedSize(800,600);
    ui.titleContainer->resize(this->width(),this->height()/7);
    ui.titleContainer->move(this->x(),this->y());
    InitBtnConnect();
    EnableBtn();
    DownLoadIniFile();
    //���ñ궨����
    SetCalibration();
    RectifyMapping();
}

BLcamera::~BLcamera()
{
    if (m_cameraStatue.bOpenDevice)
    {
        ClickClose();
    }
    //Save2Ini();
}

void BLcamera::mousePressEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton)
    {
        //�������ֵ
        //��ǰ�������-�������Ͻ�����
        p = e->globalPos() - this->frameGeometry().topLeft();
    }
}

void BLcamera::mouseMoveEvent(QMouseEvent* e)
{
    if (e->buttons() & Qt::LeftButton)
    {
        //�Ƶ����Ͻ�
        move(e->globalPos() - p);
    }
}
/*
* ��ť���źŲ����ӹ���ȫ���ڸú����ڳ�ʼ��
*/
void BLcamera::InitBtnConnect()
{
    //�رհ�ť
    QPushButton* closeBtn = this->findChild<QPushButton*>("closeBtn");
    connect(closeBtn, &QPushButton::clicked, this, [=]() {
        this->close();
        });
    //��С����ť
    QPushButton* minimizeBtn = this->findChild<QPushButton*>("minimizeBtn");
    connect(minimizeBtn, &QPushButton::clicked, this, [=]() {
        this->showMinimized();;
        });
    //���ð�ť
    QPushButton* settingBtn = this->findChild<QPushButton*>("settingBtn");
    QStackedWidget* contentStack = this->findChild<QStackedWidget*>("contentStack");
    connect(settingBtn, &QPushButton::clicked, contentStack, [=]() {
        contentStack->setCurrentIndex(1);
        });
    //���ذ�ť
    QPushButton* backBtn = this->findChild<QPushButton*>("backBtn");
    connect(backBtn, &QPushButton::clicked, contentStack, [=]() {
        //ˢ�²���
        UpdateParams();
        if (m_cameraStatue.bOpenDevice)
        {
            SetParameters();
        }
        contentStack->setCurrentIndex(0);
        });
    //���������ť
    QPushButton* cameraParamsBtn = this->findChild<QPushButton*>("cameraParamsBtn");
    QStackedWidget* stettigStack = this->findChild<QStackedWidget*>("stettigStack");
    connect(cameraParamsBtn, &QPushButton::clicked, stettigStack, [=]() {
        stettigStack->setCurrentIndex(1);
        });
    //����궨��ť
    QPushButton* cameraCalibrationBtn = this->findChild<QPushButton*>("cameraCalibrationBtn");
    connect(cameraCalibrationBtn, &QPushButton::clicked, stettigStack, [=]() {
        stettigStack->setCurrentIndex(0);
        });
    //�ɼ�ģʽ��ť
    QPushButton* grabBtn = this->findChild<QPushButton*>("grabBtn");
    connect(grabBtn, &QPushButton::clicked, stettigStack, [=]() {
        stettigStack->setCurrentIndex(2);
        });
    //ͼƬ��ʽ��ť
    QPushButton* imageBtn = this->findChild<QPushButton*>("imageBtn");
    connect(imageBtn, &QPushButton::clicked, stettigStack, [=]() {
        stettigStack->setCurrentIndex(3);
        });
    //��ʾ���ð�ť
    QPushButton* displaySettingBtn = this->findChild<QPushButton*>("displaySettingBtn");
    connect(displaySettingBtn, &QPushButton::clicked, stettigStack, [=]() {
        stettigStack->setCurrentIndex(4);
        });
    //��˫����л���ť
    QStackedWidget* cameraDispaly = this->findChild<QStackedWidget*>("cameraDispaly");
    QRadioButton* singleRadioBtn = this->findChild<QRadioButton*>("singleRadioBtn");
    connect(singleRadioBtn, &QRadioButton::clicked, cameraDispaly, [=]() {
        cameraDispaly->setCurrentIndex(1);
        });
    QRadioButton* doubleRadioBtn = this->findChild<QRadioButton*>("doubleRadioBtn");
    connect(doubleRadioBtn, &QRadioButton::clicked, cameraDispaly, [=]() {
        cameraDispaly->setCurrentIndex(0);
        });
    /**********************����������ܰ�ť*******************************/
    //��ʼ����ť
    QPushButton* iniBtn = this->findChild<QPushButton*>("iniBtn");
    connect(iniBtn, &QPushButton::clicked, this, [=]() {
        InitCamera();
        });
    //�ɼ���ť
    QPushButton* startGrabBtn = this->findChild<QPushButton*>("startGrabBtn");
    connect(startGrabBtn, &QPushButton::clicked, [=]() {
        ClickStartGrabbing();
        });
    //�ر����
    QPushButton* closeCameraBtn = this->findChild<QPushButton*>("closeCameraBtn");
    connect(closeCameraBtn, &QPushButton::clicked, [=]() {
        int nRet = ClickClose();
        if (MV_OK != nRet)
        {
            ShowErrorMsg("Close Failed!",nRet);
        }
        });
    //����ͼƬ
    QPushButton* saveImageBtn = this->findChild<QPushButton*>("saveImageBtn");
    connect(saveImageBtn, &QPushButton::clicked, [=]() {
        ClickSave();
        });
    //��ʼ���
    QPushButton* measureBtn = this->findChild<QPushButton*>("measureBtn");
    connect(measureBtn, &QPushButton::clicked, [=]() {
        ClickMeasure();
        });
    //sendData
    QPushButton* sendDataBtn = this->findChild<QPushButton*>("sendDataBtn");
    connect(sendDataBtn, &QPushButton::clicked, [=]() {
        ClickSendData();
        });

    /**************************����ģʽ����***************************************/

    //����ģʽ��ť
    QRadioButton* softRadioBtn = this->findChild<QRadioButton*>("softRadioBtn");
    connect(softRadioBtn, &QRadioButton::clicked, this, [=]() {
        //ClickTriggerModeRadio();�ݲ�֧�ִ���ģʽ
        });
    //����ģʽ��ť
    QRadioButton* ConRadioBtn = this->findChild<QRadioButton*>("ConRadioBtn");
    connect(ConRadioBtn, &QRadioButton::clicked, this, [=]() {
        ClickConModeRadio();
        });

    /**************************����ͼƬ***************************************/

    QRadioButton* radioBMP = this->findChild<QRadioButton*>("radioBMP");
    connect(radioBMP, &QRadioButton::clicked, this, [=]() {
        ClickSave2BMP();
        });
    QRadioButton* radioPNG = this->findChild<QRadioButton*>("radioPNG");
    connect(radioPNG, &QRadioButton::clicked, this, [=]() {
        ClickSave2PNG();
        });
    QRadioButton* radioTIFF = this->findChild<QRadioButton*>("radioTIFF");
    connect(radioTIFF, &QRadioButton::clicked, this, [=]() {
        ClickSave2TIFF();
        });
    QRadioButton* radioJPG = this->findChild<QRadioButton*>("radioJPG");
    connect(radioJPG, &QRadioButton::clicked, this, [=]() {
        ClickSave2JPG();
        });

    /**************************�����������***************************************/

    //��������༭����Ӧ
    QLineEdit* exposureTime = this->findChild<QLineEdit*>("exposureTime");
    connect(exposureTime, &QLineEdit::textChanged, [=](QString text) {
        m_cameraStatue.dExposureTime = text.toDouble();
        });
    QLineEdit* gain = this->findChild<QLineEdit*>("gain");
    connect(gain, &QLineEdit::textChanged, [=](QString text) {
        m_cameraStatue.dGain = text.toDouble();
        });
    QLineEdit* frameRate = this->findChild<QLineEdit*>("frameRate");
    connect(frameRate, &QLineEdit::textChanged, [=](QString text) {
        m_cameraStatue.dFrameRate = text.toDouble();
        });
}

void BLcamera::InitCamera()
{
    int nRet = EnumDevices();
    if(MV_OK != nRet)
    {
        return;
    }
    QLabel* labelCamNum = this->findChild<QLabel*>("labelCamNum");
    labelCamNum->setText("�����������"+QString::number(m_cameraStatue.nSelectDeviceNum));
    nRet = OpenDevices();
    if (MV_OK != nRet)
    {
        ShowErrorMsg("Open Fail", nRet);
        return;
    }
    QLabel* labelStatus = this->findChild<QLabel*>("labelStatus");
    labelStatus->setText("���״̬����");
    //�����ļ�����
    /*nRet = DownLoadIniFile();
    if (MV_OK != nRet)
    {
        ShowErrorMsg("DownLoad Fail", nRet);
        return;
    }*/
    //�����������
    SetParameters();
    ClickSave2JPG();
    
    
    //RectifyMapping();
    EnableBtn();
}

void BLcamera::RectifyMapping()
{
    // ͼ��У������ת����R ͶӰ����P ��ͶӰ����Q
    cv::Mat R1, R2, P1, P2, Q;
    // ͼ��У��֮�󣬻��ͼ����вü���validROIָ�ü������Ч����
    cv::Rect validROI1, validROI2;
    cv::Mat mapCam1[2], mapCam2[2];
    cv::Size imageSize = cv::Size(1280, 1024);

    cv::stereoRectify(m_cameraStatue.stereoParams.IntrinsicMatrixCam1,
        m_cameraStatue.stereoParams.DistortionMatrixCam1, m_cameraStatue.stereoParams.IntrinsicMatrixCam2,
        m_cameraStatue.stereoParams.DistortionMatrixCam2, imageSize, 
        m_cameraStatue.stereoParams.RotationMatrixOfCam2,
        m_cameraStatue.stereoParams.TranslationMatrixOfCam2,
        R1, R2, P1, P2, Q,cv::CALIB_ZERO_DISPARITY, -1, imageSize, &validROI1, &validROI2);

    cv::initUndistortRectifyMap(m_cameraStatue.stereoParams.IntrinsicMatrixCam1,
        m_cameraStatue.stereoParams.DistortionMatrixCam1,
        R1, P1, imageSize, CV_32FC1, mapCam1[0], mapCam1[1]);

    cv::initUndistortRectifyMap(m_cameraStatue.stereoParams.IntrinsicMatrixCam2,
        m_cameraStatue.stereoParams.DistortionMatrixCam2,
        R2, P2, imageSize, CV_32FC1, mapCam2[0], mapCam2[1]);
    //����
    /*qDebug()<< mapCam1[0].type();
    qDebug()<< mapCam1[1].type();
    qDebug()<< mapCam2[0].type();
    qDebug()<< mapCam2[1].type();*/
    for (int i = 0; i < 2; i++)
    {
        m_cameraStatue.stereoParams.RectifyMapCam1[i].upload(mapCam1[i]);
        m_cameraStatue.stereoParams.RectifyMapCam2[i].upload(mapCam2[i]);
    }
}

void BLcamera::ClickTriggerModeRadio()
{
    QRadioButton* softRadioBtn = this->findChild<QRadioButton*>("softRadioBtn");
    softRadioBtn->setChecked(true);
    m_cameraStatue.nTriggerMode = MV_TRIGGER_MODE_ON;
}

void BLcamera::ClickConModeRadio()
{
    QRadioButton* ConRadioBtn = this->findChild<QRadioButton*>("ConRadioBtn");
    ConRadioBtn->setChecked(true);
    m_cameraStatue.nTriggerMode = MV_TRIGGER_MODE_OFF;
}

void BLcamera::ClickStartGrabbing()
{
    QPushButton* startGrabBtn = this->findChild<QPushButton*>("startGrabBtn");
    QString str = startGrabBtn->text();
    if (QString::fromUtf8("��ʼ�ɼ�")==str)
    {
        int camera_Index = 0;
        QLabel* cameraDisplay_L = this->findChild<QLabel*>("cameraDisplay2");
        QLabel* cameraDisplay_R = this->findChild<QLabel*>("cameraDisplay1");
        QLineEdit* recFrame_L = this->findChild<QLineEdit*>("recFrame1");
        QLineEdit* lostFrame_L = this->findChild<QLineEdit*>("lostFrame1");
        QLineEdit* recFrame_R = this->findChild<QLineEdit*>("recFrame2");
        QLineEdit* lostFrame_R = this->findChild<QLineEdit*>("lostFrame2");
        if (m_cameraStatue.nTriggerMode == TRIGGER_OFF)
        {
            // ch:��ʼ�ɼ�֮��Ŵ���workthread�߳� | en:Create workthread after start grabbing
            for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
            {
                //��������ɼ�
                int nRet=m_myCamera[i].pcMyCamera->StartGrabbing();
                if (MV_OK != nRet)
                {
                    ShowErrorMsg("Start grabbing fail", nRet);
                    return;
                }
                camera_Index = i;
                if (camera_Index == 1)
                {
                    grabThread_LeftCamera = new GrabThread;
                    grabThread_LeftCamera->getCameraPtr(m_myCamera[camera_Index].pcMyCamera); //�̻߳�ȡ�����ָ��
                    grabThread_LeftCamera->getImagePtr(m_myCamera[camera_Index].myImage);  //�̻߳�ȡ��ͼ��ָ��
                    grabThread_LeftCamera->getCameraIndex(camera_Index); //����� Index==1
                    grabThread_LeftCamera->getDisplay(cameraDisplay_L);
                    grabThread_LeftCamera->getSaveImageMux(&(m_myCamera[camera_Index].SaveImageMux));
                    grabThread_LeftCamera->getCameraImageCache(&(m_myCamera[camera_Index].ImageCache));
                    connect(grabThread_LeftCamera, &GrabThread::DislpayFrame, [=](int a,int b) {
                        recFrame_L->setText(QString::number(a));
                        lostFrame_L->setText(QString::number(b));
                        });
                    connect(grabThread_LeftCamera, &GrabThread::DisplayImage, [=](QImage QmyImage) {
                        QmyImage = (QmyImage).scaled(cameraDisplay_L->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
                        cameraDisplay_L->setPixmap(QPixmap::fromImage(QmyImage));
                        cameraDisplay_L->show();
                        });
                    if (!grabThread_LeftCamera->isRunning())
                    {
                        grabThread_LeftCamera->start();
                    }

                }

                if (camera_Index == 0)
                {
                    grabThread_RightCamera = new GrabThread;
                    grabThread_RightCamera->getCameraPtr(m_myCamera[camera_Index].pcMyCamera); //�̻߳�ȡ�����ָ��
                    grabThread_RightCamera->getImagePtr(m_myCamera[camera_Index].myImage);  //�̻߳�ȡ��ͼ��ָ��
                    grabThread_RightCamera->getCameraIndex(camera_Index); //����� Index==1
                    grabThread_RightCamera->getDisplay(cameraDisplay_R);   
                    grabThread_RightCamera->getSaveImageMux(&(m_myCamera[camera_Index].SaveImageMux));
                    grabThread_RightCamera->getCameraImageCache(&(m_myCamera[camera_Index].ImageCache));
                    connect(grabThread_RightCamera, &GrabThread::DislpayFrame, [=](int a, int b) {
                        recFrame_R->setText(QString::number(a));
                        lostFrame_R->setText(QString::number(b));
                        });
                    connect(grabThread_RightCamera, &GrabThread::DisplayImage, [=](QImage QmyImage) {
                        QmyImage = (QmyImage).scaled(cameraDisplay_R->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
                        cameraDisplay_R->setPixmap(QPixmap::fromImage(QmyImage));
                        cameraDisplay_R->show();
                        });
                    if (!grabThread_RightCamera->isRunning())
                    {
                        grabThread_RightCamera->start();
                    }
                }
            }
        }
        m_cameraStatue.bStartGrabbing = true;
        QLabel* labelStatus = this->findChild<QLabel*>("labelStatus");
        labelStatus->setText("���״̬���ɼ���");
        startGrabBtn->setText("ֹͣ�ɼ�");
    }
    if (QString::fromUtf8("ֹͣ�ɼ�") == str)
    {
        for (unsigned int i = 0; i < m_cameraStatue.nSelectDeviceNum; i++)
        {
            if ( (0 == i) && grabThread_LeftCamera->isRunning())
            {
                m_myCamera[i].pcMyCamera->StopGrabbing();
                grabThread_LeftCamera->requestInterruption();
                grabThread_LeftCamera->wait();
            }
            if ((1 == i) && grabThread_RightCamera->isRunning())
            {
                m_myCamera[i].pcMyCamera->StopGrabbing();
                grabThread_RightCamera->requestInterruption();
                grabThread_RightCamera->wait();
            }
        }
        m_cameraStatue.bStartGrabbing = false;
        QLabel* labelStatus = this->findChild<QLabel*>("labelStatus");
        labelStatus->setText("���״̬����");
        startGrabBtn->setText("��ʼ�ɼ�");
    }
    EnableBtn();
}

void BLcamera::ClickMeasure()
{
    QPushButton* measureBtn = this->findChild<QPushButton*>("measureBtn");
    QString str = measureBtn->text();
    if (QString::fromUtf8("��ʼ���") == str)
    {
        m_cameraStatue.bMeasure = true;
        grabThread_RightCamera->getMeasureFlag(m_cameraStatue.bMeasure);
        grabThread_LeftCamera->getLineFlag(true);
        detect = new DetectThread;
        detect->getCameraImageCache(&(m_myCamera[0].ImageCache), &(m_myCamera[1].ImageCache));
        detect->getSaveImageMux(&(m_myCamera[0].SaveImageMux), &(m_myCamera[1].SaveImageMux));
        detect->getSps(m_cameraStatue.stereoParams);
        detect->InitMonitor(1);
        connect(detect, &DetectThread::TargetFlagSignal, grabThread_RightCamera, &GrabThread::TargetFlagSlot);
        connect(detect, &DetectThread::TargetFlagSignal, grabThread_LeftCamera, &GrabThread::TargetFlagSlot);
        connect(detect, &DetectThread::DisplaySignal, this, &BLcamera::DisplayOnRightSlot);
        connect(detect, &DetectThread::PosSignal, this, &BLcamera::DisplayPositionSlot);
        if (!detect->isRunning())
        {
            detect->start();
        }
        
        QLabel* labelStatus = this->findChild<QLabel*>("labelStatus");
        labelStatus->setText("���״̬�������");
        measureBtn->setText("ֹͣ���");
    }
    else if (QString::fromUtf8("ֹͣ���") == str)
    {
        m_cameraStatue.bMeasure = false;
        grabThread_LeftCamera->getMeasureFlag(m_cameraStatue.bMeasure);
        grabThread_RightCamera->getLineFlag(false);
        //detect->InitMonitor(0); ����ã������̱߳���
        if (detect->isRunning())
        {
            detect->requestInterruption();
            detect->wait();
        }
        QLabel* labelStatus = this->findChild<QLabel*>("labelStatus");
        labelStatus->setText("���״̬���ɼ���");
        measureBtn->setText("��ʼ���");
    }
    /*cv::namedWindow("img", WINDOW_AUTOSIZE);
    cv::imshow("img", *m_myCamera[0].myImage);
    waitKey(0);*/
}

void BLcamera::ClickSendData()
{
#if 0
    CptAgl cvCptAgl = { 0 };
    cv::cuda::GpuMat gResize1, gResize2;
    //��СͼƬ
    cv::cuda::GpuMat gImg1, gImg2;
    m_myCamera[0].SaveImageMux.lock();
    m_myCamera[1].SaveImageMux.lock();
    if ((m_myCamera[0].ImageCache.empty() || m_myCamera[1].ImageCache.empty()))
    {
        m_myCamera[0].SaveImageMux.unlock();
        m_myCamera[1].SaveImageMux.unlock();
        return;
    }
    gImg1 = m_myCamera[0].ImageCache.clone();
    gImg2 = m_myCamera[1].ImageCache.clone();
    m_myCamera[0].SaveImageMux.unlock();
    m_myCamera[1].SaveImageMux.unlock();

    cv::cuda::resize(gImg1, gResize1, cv::Size(), 0.5, 0.5);
    cv::cuda::resize(gImg2, gResize2, cv::Size(), 0.5, 0.5);
    //ͼ�ν���
    cv::cuda::GpuMat gRemap1, gRemap2;
    cv::cuda::remap(gImg1, gRemap1, m_cameraStatue.stereoParams.RectifyMapCam1[0], m_cameraStatue.stereoParams.RectifyMapCam1[1], cv::INTER_LINEAR);
    cv::cuda::remap(gImg2, gRemap2, m_cameraStatue.stereoParams.RectifyMapCam2[0], m_cameraStatue.stereoParams.RectifyMapCam2[1], cv::INTER_LINEAR);
    cv::Mat g1, g2;
    gRemap1.download(g1);
    gRemap2.download(g2);
    cv::Mat temp;
    cv::hconcat(g1, g2, temp);
    //����
    int xstart = 0;
    int xend = temp.cols;
    int ystart = 0;
    int yend = temp.rows;
    /*for (int i = 100; i >0; i = i-2)
    {
        cv::Point po = cv::Point(xstart, yend/i);
        cv::Point p1 = cv::Point(xend, yend / i);
        cv::line(temp, po, p1, cv::Scalar(255, 0, 0), 3, 4);
    }*/
    cv::Mat res;
    cv::cvtColor(temp, res, CV_GRAY2BGR);
    for (int i = 1; i < yend; i = i + 30)
    {
        cv::Point po = cv::Point(xstart, i);
        cv::Point p1 = cv::Point(xend, i);
        cv::line(res, po, p1, cv::Scalar(255, 0, 0), 3, 4);
    }
    /*cv::Point po = cv::Point(xstart, 50);
    cv::Point p1 = cv::Point(xend, 50);
    cv::line(temp, po, p1, cv::Scalar(255, 0, 0), 3, 4);*/
    cv::namedWindow("map", WINDOW_KEEPRATIO);
    cv::imshow("map", res);
    cv::waitKey(0);
#endif
#if 1
    /*m_myCamera[0].SaveImageMux.lock();
    m_myCamera[1].SaveImageMux.lock();
    cv::cuda::GpuMat img1,img2;
    img1 = m_myCamera[0].ImageCache.clone();
    img2 = m_myCamera[1].ImageCache.clone();
    m_myCamera[0].SaveImageMux.unlock();
    m_myCamera[1].SaveImageMux.unlock();*/
    //detect->FindContours(img1, 1, m_cameraStatue.stereoParams.RectifyMapCam2[0], m_cameraStatue.stereoParams.RectifyMapCam2[1], m_cameraStatue.stereoParams.RoiCam2);
    //detect->FindContours(img2, 0, m_cameraStatue.stereoParams.RectifyMapCam1[0], m_cameraStatue.stereoParams.RectifyMapCam1[1], m_cameraStatue.stereoParams.RoiCam1);
    detect->PositionAndPosture();
    
    
#endif // 1
#if 0
    Yolov5* yolo = new Yolov5();
    yolo->Initialize("myyolov5s.engine", 0);
    //��ȡ����ͼ��
    cv::Mat img;
    m_myCamera[0].SaveImageMux.lock();
    m_myCamera[0].ImageCache.download(img);
    m_myCamera[0].SaveImageMux.unlock();
    //cv::Mat img = cv::imread("images_tmp/11.jpg");
    //cv::Mat img2 = cvt23Channels(img);
    std::vector<cv::Rect> Boxes;
    std::vector<const char*> ClassLables;
    yolo->Detecting(img, Boxes, ClassLables);
    //���Խ������λ��
    cv::imwrite("jiaozheng/yuantu.jpg", img);
    cv::Point p1(Boxes[0].x, Boxes[0].y);
    cv::Mat rimg, gray;
    cv::Mat RectifyMapCam1[2];
    m_cameraStatue.stereoParams.RectifyMapCam1[0].download(RectifyMapCam1[0]);
    m_cameraStatue.stereoParams.RectifyMapCam1[1].download(RectifyMapCam1[1]);
    if (img.channels() > 1)
    {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    }
    cv::remap(img, rimg, RectifyMapCam1[0], RectifyMapCam1[1], cv::INTER_LINEAR);
    cv::imwrite("jiaozheng/remap.jpg",rimg);
    if (yolo != NULL)
    {
        delete yolo;
    }
    cv::resize(img, img, cv::Size(round(img.cols / 2), round(img.rows / 2)));
    cv::namedWindow("input", cv::WINDOW_AUTOSIZE);
    cv::imshow("input", img);
    cv::waitKey(0);
#endif // 1

}

int BLcamera::ClickClose()
{
    int nRet = MV_OK;
    if (m_cameraStatue.bStartGrabbing) {
        for (unsigned int i = 0; i < m_cameraStatue.nSelectDeviceNum; i++)
        {
            if ((0 == i) && grabThread_LeftCamera->isRunning())
            {
                m_myCamera[i].pcMyCamera->StopGrabbing();
                grabThread_LeftCamera->requestInterruption();
                grabThread_LeftCamera->wait();
            }
            if ((1 == i) && grabThread_RightCamera->isRunning())
            {
                m_myCamera[i].pcMyCamera->StopGrabbing();
                grabThread_RightCamera->requestInterruption();
                grabThread_RightCamera->wait();
            }
        }
    }
    for (int i = 0; i < m_cameraStatue.nSelectDeviceNum; i++)
    {
        if (m_myCamera[i].pcMyCamera->m_pBufForDriver)
        {
            free(m_myCamera[i].pcMyCamera->m_pBufForDriver);
            m_myCamera[i].pcMyCamera->m_pBufForDriver = NULL;
        }
        if (m_myCamera[i].pcMyCamera->m_pBufForSaveImage)
        {
            free(m_myCamera[i].pcMyCamera->m_pBufForSaveImage);
            m_myCamera[i].pcMyCamera->m_pBufForSaveImage = NULL;
        }
    }
    for (int i = 0; i < m_cameraStatue.nSelectDeviceNum; i++)
    {
        nRet = m_myCamera[i].pcMyCamera->Close();
        if (MV_OK != nRet)
        {
            return nRet;
        }
        delete m_myCamera[i].pcMyCamera;
        m_myCamera[i].pcMyCamera = NULL;
    }
    m_cameraStatue.bStartGrabbing = false;
    m_cameraStatue.bOpenDevice = false;
    m_cameraStatue.nSelectDeviceNum = 0;
    QLabel* labelCamNum = this->findChild<QLabel*>("labelCamNum");
    labelCamNum->setText("�����������0");
    QLabel* labelStatus = this->findChild<QLabel*>("labelStatus");
    labelStatus->setText("���״̬���ر�");
    return nRet;
}

int BLcamera::ClickSave()
{
    int nRet = SaveImage();
    if (MV_OK != nRet)
    {
        ShowErrorMsg("Save Failed!",nRet);
    }
    return 0;
}

int BLcamera::EnumDevices()
{
    memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // ch:ö�������������豸 | en:Enumerate all devices within subnet
    int nRet = CMvCamera::EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDevList);
    if (MV_OK != nRet)
    {
        return nRet;
    }
    if (0 == m_stDevList.nDeviceNum)
    {
        ShowErrorMsg("No device",0);
        return STATUS_ERROR;
    }
    m_cameraStatue.nSelectDeviceNum = m_stDevList.nDeviceNum;
    return MV_OK;
}

int BLcamera::OpenDevices()
{
    if (m_cameraStatue.bOpenDevice)
    {
        return STATUS_ERROR;
    }
    if (0 == m_cameraStatue.nSelectDeviceNum)
    {
        return STATUS_ERROR;
    }
    // ch:���豸��Ϣ�����豸ʵ�� | en:Device instance created by device information
    for (int i=0; i < m_cameraStatue.nSelectDeviceNum; i++)
    {
        if (NULL != m_myCamera[i].pcMyCamera)
        {
            return STATUS_ERROR;
        }
        m_myCamera[i].pcMyCamera = new CMvCamera;
        if (NULL == m_myCamera[i].pcMyCamera)
        {
            return STATUS_ERROR;
        }
        int nRet = m_myCamera[i].pcMyCamera->Open(m_stDevList.pDeviceInfo[i]);
        if (MV_OK != nRet)
        {
            delete m_myCamera[i].pcMyCamera;
            m_myCamera[i].pcMyCamera = NULL;
            return nRet;
        }
        // ch:̽��������Ѱ���С(ֻ��GigE�����Ч) | en:Detection network optimal package size(It only works for the GigE camera)
        if (m_stDevList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
        {
            unsigned int nPacketSize = 0;
            nRet = m_myCamera[i].pcMyCamera->GetOptimalPacketSize(&nPacketSize);
            if (nRet == MV_OK)
            {
                nRet = m_myCamera[i].pcMyCamera->SetIntValue("GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    ShowErrorMsg("Warning: Set Packet Size fail!", nRet);
                }
            }
            else
            {
                ShowErrorMsg("Warning: Get Packet Size fail!", nRet);
            }
        }
    }
    m_cameraStatue.bOpenDevice = true;
    return MV_OK;
}

int BLcamera::DownLoadIniFile()
{
    //�����������ģʽ
    int temp = ReadIniFile(INI_NAME,"TRIGGER_MODE","mode").toInt();
    if (MV_TRIGGER_MODE_ON == temp)
    {
        ClickTriggerModeRadio();
    }
    else if(MV_TRIGGER_MODE_OFF == temp)
    {
        ClickConModeRadio();
    }
    else
    {
        return STATUS_ERROR;
    }
    //���ش���Դ
    temp = ReadIniFile(INI_NAME, "TRIGGER_SOURCE", "source").toInt();
    if (MV_TRIGGER_SOURCE_SOFTWARE == temp)
    {
        m_cameraStatue.nTriggerSource = MV_TRIGGER_SOURCE_SOFTWARE;
    }
    else
    {
        return STATUS_ERROR;
    }
    //�����ع�
    QVariant value = ReadIniFile(INI_NAME, "CAMERA_PARAMS", "exposureTime");
    QLineEdit* exposureTime = this->findChild<QLineEdit*>("exposureTime");
    exposureTime->setText(value.toString());
    //��������
    value = ReadIniFile(INI_NAME, "CAMERA_PARAMS", "gain");
    QLineEdit* gain = this->findChild<QLineEdit*>("gain");
    gain->setText(value.toString());
    //����֡��
    value = ReadIniFile(INI_NAME, "CAMERA_PARAMS", "frameRate");
    QLineEdit* frameRate = this->findChild<QLineEdit*>("frameRate");
    frameRate->setText(value.toString());

    /******************************��������궨����*************************************/
    //�ڲξ���
    QList<QString> strList = {
    "11","12","13",
    "21","22","23",
    "31","32","33"};
    for (int i = 0,k=0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            value = ReadIniFile(INI_NAME, INTMATIRXCAM1,strList.at(k));
            m_cameraStatue.stereoParams.IntrinsicMatrixCam1.at<double>(i, j) = value.toDouble();
            value = ReadIniFile(INI_NAME, INTMATIRXCAM2,strList.at(k));
            m_cameraStatue.stereoParams.IntrinsicMatrixCam2.at<double>(i, j) = value.toDouble();
            value = ReadIniFile(INI_NAME, ROTATIONOFCAM2,strList.at(k));
            m_cameraStatue.stereoParams.RotationMatrixOfCam2.at<double>(i, j) = value.toDouble();
            k++;
        }
    }
    for (int i = 0; i < 3; i++)
    {
        value = ReadIniFile(INI_NAME, TRANSOFCAM2, strList.at(i));
        m_cameraStatue.stereoParams.TranslationMatrixOfCam2.at<double>(i) = value.toDouble();
    }
    strList = {
    "k1","k2",
    "p1","p2"};
    for (int i = 0; i < 4; i++)
    {
            value = ReadIniFile(INI_NAME, DISMATRIXCAM1, strList.at(i));
            m_cameraStatue.stereoParams.DistortionMatrixCam1.at<double>(i) = value.toDouble();
            value = ReadIniFile(INI_NAME, DISMATRIXCAM2, strList.at(i));
            m_cameraStatue.stereoParams.DistortionMatrixCam2.at<double>(i) = value.toDouble();
    }
    strList = { "L","T","W","H" };
    for (int i = 0; i < 4; i++)
    {
        value = ReadIniFile(INI_NAME, ROICAM1, strList.at(i));
        m_cameraStatue.stereoParams.RoiCam1.at<double>(i) = value.toDouble();
        value = ReadIniFile(INI_NAME, ROICAM2, strList.at(i));
        m_cameraStatue.stereoParams.RoiCam2.at<double>(i) = value.toDouble();
    }
    return MV_OK;
}

int BLcamera::SetParameters()
{
    int nRet = SetExposureTime();
    if (nRet != MV_OK)
    {
        ShowErrorMsg("Set Exposure Time Fail", nRet);
    }
    nRet = SetGain();
    if (nRet != MV_OK)
    {
        ShowErrorMsg("Set Gain Fail", nRet);
    }
    nRet = SetFrameRate();
    if (nRet != MV_OK)
    {
        ShowErrorMsg("Set Frame Rate Fail", nRet);
    }
    return 0;
}

int BLcamera::SetExposureTime()
{
    for (int i=0; i < m_cameraStatue.nSelectDeviceNum; i++) {
        int nRet = m_myCamera[i].pcMyCamera->SetEnumValue("ExposureMode", MV_EXPOSURE_MODE_TIMED);
        if (MV_OK != nRet)
        {
            return nRet;
        }

        nRet = m_myCamera[i].pcMyCamera->SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);

        nRet = m_myCamera[i].pcMyCamera->SetFloatValue("ExposureTime", (float)m_cameraStatue.dExposureTime);
        if (MV_OK != nRet)
        {
            return nRet;
        }
    }
    return MV_OK;
}

int BLcamera::SetGain()
{
    // ch:��������ǰ�Ȱ��Զ�����رգ�ʧ�����践��
    //en:Set Gain after Auto Gain is turned off, this failure does not need to return
    for (int i=0; i < m_cameraStatue.nSelectDeviceNum; i++) {
        int nRet = m_myCamera[i].pcMyCamera->SetEnumValue("GainAuto", 0);
        if (MV_OK != nRet)
        {
            return nRet;
        }
        nRet = m_myCamera[i].pcMyCamera->SetFloatValue("Gain", (float)m_cameraStatue.dGain);
        if (MV_OK != nRet)
        {
            return nRet;
        }
    }
    return MV_OK;
}

int BLcamera::SetFrameRate()
{
    for (int i=0; i < m_cameraStatue.nSelectDeviceNum; i++) {
        int nRet = m_myCamera[i].pcMyCamera->SetBoolValue("AcquisitionFrameRateEnable", true);
        if (MV_OK != nRet)
        {
            return nRet;
        }
        nRet = m_myCamera[i].pcMyCamera->SetFloatValue("AcquisitionFrameRate", (float)m_cameraStatue.dFrameRate);
        if (MV_OK != nRet)
        {
            return nRet;
        }
    }
    return MV_OK;
}

int BLcamera::StartGrabbing()
{
    if (false == m_cameraStatue.bOpenDevice || true == m_cameraStatue.bStartGrabbing)
    {
        return STATUS_ERROR;
    }

    return 0;
}

void BLcamera::EnableBtn()
{
    QPushButton* startGrabBtn = this->findChild<QPushButton*>("startGrabBtn");
    QPushButton* saveImageBtn = this->findChild<QPushButton*>("saveImageBtn");
    QPushButton* measureBtn = this->findChild<QPushButton*>("measureBtn");
    QPushButton* closeCameraBtn = this->findChild<QPushButton*>("closeCameraBtn");
    QPushButton* sendDataBtn = this->findChild<QPushButton*>("sendDataBtn");
    startGrabBtn->setEnabled(m_cameraStatue.bOpenDevice);
    saveImageBtn->setEnabled(m_cameraStatue.bStartGrabbing);
    measureBtn->setEnabled(m_cameraStatue.bStartGrabbing);
    sendDataBtn->setEnabled(m_cameraStatue.bStartGrabbing);
    closeCameraBtn->setEnabled(m_cameraStatue.bOpenDevice);
}

int BLcamera::SaveImage()
{
    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned int nDataLen = 0;
    int nRet = MV_OK;
    for (int i = 0; i < m_cameraStatue.nSelectDeviceNum; i++)
    {
        // ch:���ڵ�һ�α���ͼ��ʱ���뻺�棬�� CloseDevice ʱ�ͷ�
        // en:Request buffer first time save image, release after CloseDevice
        if (NULL == m_myCamera[i].pcMyCamera->m_pBufForDriver)
        {
            unsigned int nRecvBufSize = 0;
            unsigned int nRet = m_myCamera[i].pcMyCamera->GetIntValue("PayloadSize", &nRecvBufSize);

            m_myCamera[i].pcMyCamera->m_nBufSizeForDriver = nRecvBufSize;  // һ֡���ݴ�С
            m_myCamera[i].pcMyCamera->m_pBufForDriver = (unsigned char*)malloc(m_myCamera[i].pcMyCamera->m_nBufSizeForDriver);
        }

        nRet = m_myCamera[i].pcMyCamera->GetOneFrameTimeout(m_myCamera[i].pcMyCamera->m_pBufForDriver, &nDataLen, m_myCamera[i].pcMyCamera->m_nBufSizeForDriver, &stImageInfo, 1000);
        if (MV_OK == nRet)
        {
            // ch:���ڵ�һ�α���ͼ��ʱ���뻺�棬�� CloseDevice ʱ�ͷ�
            // en:Request buffer first time save image, release after CloseDevice
            if (NULL == m_myCamera[i].pcMyCamera->m_pBufForSaveImage)
            {
                // ch:BMPͼƬ��С��width * height * 3 + 2048(Ԥ��BMPͷ��С)
                // en:BMP image size: width * height * 3 + 2048 (Reserved BMP header size)
                m_myCamera[i].pcMyCamera->m_nBufSizeForSaveImage = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;

                m_myCamera[i].pcMyCamera->m_pBufForSaveImage = (unsigned char*)malloc(m_myCamera[i].pcMyCamera->m_nBufSizeForSaveImage);

            }
            MV_SAVE_IMG_TO_FILE_PARAM pstSaveFileParam;
            memset(&pstSaveFileParam, 0, sizeof(MV_SAVE_IMG_TO_FILE_PARAM));
            pstSaveFileParam.enImageType = m_cameraStatue.SaveImageType; // ch:��Ҫ�����ͼ������ | en:Image format to save
            pstSaveFileParam.enPixelType = stImageInfo.enPixelType;  // ch:�����Ӧ�����ظ�ʽ | en:Camera pixel type
            pstSaveFileParam.nWidth = stImageInfo.nWidth;         // ch:�����Ӧ�Ŀ� | en:Width
            pstSaveFileParam.nHeight = stImageInfo.nHeight;          // ch:�����Ӧ�ĸ� | en:Height
            pstSaveFileParam.nDataLen = stImageInfo.nFrameLen;
            pstSaveFileParam.pData = m_myCamera[i].pcMyCamera->m_pBufForDriver;

            // ch:jpgͼ��������ΧΪ(50-99], pngͼ��������ΧΪ[0-9] | en:jpg image nQuality range is (50-99], png image nQuality range is [0-9]
            if (m_cameraStatue.SaveImageType == MV_Image_Jpeg)
            {
                pstSaveFileParam.nQuality = 80;
            }
            else if (m_cameraStatue.SaveImageType == MV_Image_Png)
            {
                pstSaveFileParam.nQuality = 8;
            }
            unsigned int nFrameNum = stImageInfo.nFrameNum;
            pstSaveFileParam.iMethodValue = 0;
            char chImageName[IMAGE_NAME_LEN] = { 0 };
            if (MV_Image_Bmp == pstSaveFileParam.enImageType)
            {
                if (i == 0) {
                    sprintf_s(chImageName, IMAGE_NAME_LEN, "./captureImages/Left/Image_w%d_h%d_fn%03d.bmp", pstSaveFileParam.nWidth, pstSaveFileParam.nHeight, nFrameNum);
                }
                else if (i == 1) {
                    sprintf_s(chImageName, IMAGE_NAME_LEN, "./captureImages/Right/Image_w%d_h%d_fn%03d.bmp", pstSaveFileParam.nWidth, pstSaveFileParam.nHeight, nFrameNum);
                }
            }
            else if (MV_Image_Jpeg == pstSaveFileParam.enImageType)
            {
                if (i == 0) {
                    sprintf_s(chImageName, IMAGE_NAME_LEN, "./captureImages/Left/Image_w%d_h%d_fn%03d.jpg", pstSaveFileParam.nWidth, pstSaveFileParam.nHeight, nFrameNum);
                }
                else if (i == 1) {
                    sprintf_s(chImageName, IMAGE_NAME_LEN, "./captureImages/Right/Image_w%d_h%d_fn%03d.jpg", pstSaveFileParam.nWidth, pstSaveFileParam.nHeight, nFrameNum);
                }
            }
            else if (MV_Image_Tif == pstSaveFileParam.enImageType)
            {
                if (i == 0) {
                    sprintf_s(chImageName, IMAGE_NAME_LEN, "./captureImages/Left/Image_w%d_h%d_fn%03d.tif", pstSaveFileParam.nWidth, pstSaveFileParam.nHeight, nFrameNum);
                }
                else if (i == 1) {
                    sprintf_s(chImageName, IMAGE_NAME_LEN, "./captureImages/Right/Image_w%d_h%d_fn%03d.tif", pstSaveFileParam.nWidth, pstSaveFileParam.nHeight, nFrameNum);
                }
            }
            else if (MV_Image_Png == pstSaveFileParam.enImageType)
            {
                if (i == 0) {
                    sprintf_s(chImageName, IMAGE_NAME_LEN, "./captureImages/Left/Image_w%d_h%d_fn%03d.png", pstSaveFileParam.nWidth, pstSaveFileParam.nHeight, nFrameNum);
                }
                else if (i == 1) {
                    sprintf_s(chImageName, IMAGE_NAME_LEN, "./captureImages/Right/Image_w%d_h%d_fn%03d.png", pstSaveFileParam.nWidth, pstSaveFileParam.nHeight, nFrameNum);
                }
            }
            memcpy(pstSaveFileParam.pImagePath, chImageName, IMAGE_NAME_LEN);
            int nRet = m_myCamera[i].pcMyCamera->SaveImageToFile(&pstSaveFileParam);
            //����
            /*cv::Rect rec = cv::Rect(200, 100, 900, 100);
            for (int i = 0; i < 2; i++)
            {
                cv::cuda::GpuMat m;
                m_myCamera[i].SaveImageMux.lock();
                m = m_myCamera[i].ImageCache.clone();
                m_myCamera[i].SaveImageMux.unlock();
                cv::Mat mm;
                m.download(mm);
                if (!(mm.channels() > 1))
                {
                    cv::cvtColor(mm, mm, CV_GRAY2BGR);
                }
                cv::Point p1 = cv::Point(rec.x, rec.y);
                cv::Point p2 = cv::Point(rec.x + rec.width, rec.y);
                cv::Point p3 = cv::Point(rec.x , rec.y+rec.height);
                cv::Point p4 = cv::Point(rec.x + rec.width, rec.y + rec.height);
                cv::line(mm, p1, p2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
                cv::line(mm, p3, p4, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
                if (i == 0)
                {
                    cv::imwrite("temp/left.jpg", mm);
                }
                else
                {
                    cv::imwrite("temp/right.jpg", mm);
                }
                
            }*/
            if (MV_OK != nRet)
            {
                return STATUS_ERROR;
            }
        }
    }
    return nRet;
}

void BLcamera::UpdateParams()
{
    QLineEdit* edit;
    edit = this->findChild<QLineEdit*>("exposureTime");
    m_cameraStatue.dExposureTime =  edit->text().toDouble();
    edit = this->findChild<QLineEdit*>("gain");
    m_cameraStatue.dGain = edit->text().toDouble();
    edit = this->findChild<QLineEdit*>("frameRate");
    m_cameraStatue.dFrameRate = edit->text().toDouble();
    //�����ڲ�  ��ʱ��֧�ָ������
    /*QLineEdit* tempEdit;
    QList<QString> temp1= {"cam1InParams11","cam1InParams12", "cam1InParams13", 
    "cam1InParams21", "cam1InParams22", "cam1InParams23", 
    "cam1InParams31", "cam1InParams32", "cam1InParams33", };
    for (int i = 0,k=0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            tempEdit = this->findChild<QLineEdit*>(temp1.at(k));
            k++;
            QString str = tempEdit->text();
            m_cameraStatue.stereoParams.IntrinsicMatrixCam1.at<double>(i, j) = str.toDouble();
        }
    }
    QList<QString> temp2 = { "cam2InParams11","cam2InParams12", "cam2InParams13",
    "cam2InParams21", "cam2InParams22", "cam2InParams23",
    "cam2InParams31", "cam2InParams32", "cam2InParams33", };
    for (int i = 0,k=0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            tempEdit = this->findChild<QLineEdit*>(temp2.at(k));
            k++;
            QString str = tempEdit->text();
            m_cameraStatue.stereoParams.IntrinsicMatrixCam2.at<double>(i, j) = str.toDouble();
        }
    }*/
    //�������  
    //QList<QString> temp3 = { "RotationMatrixOfCam2_11","RotationMatrixOfCam2_12", "RotationMatrixOfCam2_13",
    //"RotationMatrixOfCam2_21", "RotationMatrixOfCam2_22", "RotationMatrixOfCam2_23",
    //"RotationMatrixOfCam2_31", "RotationMatrixOfCam2_32", "RotationMatrixOfCam2_33", };
    //for (int i = 0, k = 0; i < 3; i++)
    //{
    //    for (int j = 0; j < 3; j++)
    //    {
    //        tempEdit = this->findChild<QLineEdit*>(temp3.at(k));
    //        k++;
    //        QString str = tempEdit->text();
    //        m_cameraStatue.stereoParams.RotationMatrixOfCam2.at<double>(i, j) = str.toDouble();
    //    }
    //}
    //QList<QString> temp4 = { "TranslationMatrixOfCam2_1","TranslationMatrixOfCam2_2", "TranslationMatrixOfCam2_3"};
    //for (int i = 0; i < 3; i++)
    //{
    //    tempEdit = this->findChild<QLineEdit*>(temp4.at(i));
    //    QString str = tempEdit->text();
    //    m_cameraStatue.stereoParams.TranslationMatrixOfCam2.at<double>(i) = str.toDouble();
    //}
    ////���»���
    //QList<QString> temp5 = { "cam1k1","cam1p1", "cam1k2" ,"cam1p2" ,"cam2k1","cam2p1","cam2k2","cam2p2" };
    //int k = 0;
    //for (int i = 0; i < 2; i++)
    //{
    //    for (int j = 0; j < 2; j++)
    //    {
    //        tempEdit = this->findChild<QLineEdit*>(temp5.at(k));
    //        k++;
    //        QString str = tempEdit->text();
    //        m_cameraStatue.stereoParams.DistortionMatrixCam1.at<double>(i,j) = str.toDouble();
    //    }
    //}
    //for (int i = 0; i < 2; i++)
    //{
    //    for (int j = 0; j < 2; j++)
    //    {
    //        tempEdit = this->findChild<QLineEdit*>(temp5.at(k));
    //        k++;
    //        QString str = tempEdit->text();
    //        m_cameraStatue.stereoParams.DistortionMatrixCam2.at<double>(i, j) = str.toDouble();
    //    }
    //}
}

void BLcamera::Save2Ini()
{
    //�ڲξ��󱣴�
    QList<IniData> cam1IntrinsicMatrix;
    QList<IniData> cam2IntrinsicMatrix;
    //��ξ��󱣴�
    QList<IniData> RotationMatrixOfCam2;
    QList<IniData> TranslationMatrixOfCam2;
    QList<QString> temp = {"11","12" ,"13" ,
    "21" ,"22" ,"23" ,
    "31" ,"32" ,"33" };
    for (int i =0,k=0; i< m_cameraStatue.stereoParams.IntrinsicMatrixCam1.rows;i++)
    {
        TranslationMatrixOfCam2.append(IniData(temp.at(i), QString::number(m_cameraStatue.stereoParams.TranslationMatrixOfCam2.at<double>(i), 'f', 4)));
        for (int j = 0; j < m_cameraStatue.stereoParams.IntrinsicMatrixCam1.cols; j++)
        {
            cam1IntrinsicMatrix.append(IniData(temp.at(k),QString::number(m_cameraStatue.stereoParams.IntrinsicMatrixCam1.at<double>(i,j),'f', 4)));
            cam2IntrinsicMatrix.append(IniData(temp.at(k),QString::number(m_cameraStatue.stereoParams.IntrinsicMatrixCam2.at<double>(i,j), 'f', 4)));
            RotationMatrixOfCam2.append(IniData(temp.at(k),QString::number(m_cameraStatue.stereoParams.RotationMatrixOfCam2.at<double>(i,j), 'f', 4)));
            k++;
        }
    }
    WriteIniFile(INI_NAME,INTMATIRXCAM1, cam1IntrinsicMatrix);
    WriteIniFile(INI_NAME, INTMATIRXCAM2, cam2IntrinsicMatrix);
    WriteIniFile(INI_NAME,ROTATIONOFCAM2, RotationMatrixOfCam2);
    WriteIniFile(INI_NAME,TRANSOFCAM2, TranslationMatrixOfCam2); 

    //������󱣴�
    QList<IniData> DistortionMatrixCam1;
    QList<IniData> DistortionMatrixCam2;
    QList<QString> temp2 = {"k1","p1","k2","p2"};
    for (int i = 0,k=0; i < m_cameraStatue.stereoParams.DistortionMatrixCam1.rows; i++)
    {
        for (int j = 0; j < m_cameraStatue.stereoParams.DistortionMatrixCam1.cols; j++)
        {
            DistortionMatrixCam1.append(IniData(temp2.at(k),QString::number( m_cameraStatue.stereoParams.DistortionMatrixCam1.at<double>(i,j), 'f', 6)));
            DistortionMatrixCam2.append(IniData(temp2.at(k),QString::number( m_cameraStatue.stereoParams.DistortionMatrixCam2.at<double>(i,j), 'f', 6)));
            k++;
        }
    }
    WriteIniFile(INI_NAME, DISMATRIXCAM1, DistortionMatrixCam1);
    WriteIniFile(INI_NAME, DISMATRIXCAM2, DistortionMatrixCam2);
}

void BLcamera::SetCalibration()
{
    QLineEdit* edit;
    QList<QString> temp1 = {
    "cam1InParams11","cam1InParams12","cam1InParams13",
    "cam1InParams21","cam1InParams22","cam1InParams23",
    "cam1InParams31","cam1InParams32","cam1InParams33"};
    QList<QString> temp2 = { 
    "cam2InParams11","cam2InParams12","cam2InParams13",
    "cam2InParams21","cam2InParams22","cam2InParams23",
    "cam2InParams31","cam2InParams32","cam2InParams33" };
    QList<QString> temp3 = { 
    "RotationMatrixOfCam2_11","RotationMatrixOfCam2_12","RotationMatrixOfCam2_13",
    "RotationMatrixOfCam2_21","RotationMatrixOfCam2_22","RotationMatrixOfCam2_23",
    "RotationMatrixOfCam2_31","RotationMatrixOfCam2_32","RotationMatrixOfCam2_33" };
    QString str;
    for (int i = 0, k = 0; i < 3 ;i++)
    {
        for (int j = 0; j < 3; j++)
        {
            edit = this->findChild<QLineEdit*>(temp1.at(k));
            str =QString::number( m_cameraStatue.stereoParams.IntrinsicMatrixCam1.at<double>(i, j),'f',4);
            edit->setText(str);
            edit = this->findChild<QLineEdit*>(temp2.at(k));
            str = QString::number(m_cameraStatue.stereoParams.IntrinsicMatrixCam2.at<double>(i, j), 'f', 4);
            edit->setText(str);
            QString a = temp3.at(k);
            edit = this->findChild<QLineEdit*>(temp3.at(k));
            str = QString::number(m_cameraStatue.stereoParams.RotationMatrixOfCam2.at<double>(i, j), 'f', 4);
            edit->setText(str);
            k++;
        }
    }
    QList<QString> temp4 = {"TranslationMatrixOfCam2_1","TranslationMatrixOfCam2_2","TranslationMatrixOfCam2_3"};
    for (int i = 0; i < 3; i++)
    {
        edit = this->findChild<QLineEdit*>(temp4.at(i));
        str = QString::number(m_cameraStatue.stereoParams.TranslationMatrixOfCam2.at<double>(i), 'f', 4);
        edit->setText(str);
    }
    QList<QString> temp5 = {"cam1k1","cam1k2",
        "cam1p1","cam1p2"};
    QList<QString> temp6 = { "cam2k1","cam2k2",
        "cam2p1","cam2p2" };
    for (int i = 0; i < 4; i++)
    {
            edit = this->findChild<QLineEdit*>(temp5.at(i));
            str = QString::number(m_cameraStatue.stereoParams.DistortionMatrixCam1.at<double>(i), 'f', 6);
            edit->setText(str);
            edit = this->findChild<QLineEdit*>(temp6.at(i));
            str = QString::number(m_cameraStatue.stereoParams.DistortionMatrixCam2.at<double>(i), 'f', 6);
            edit->setText(str);
    }
}

void BLcamera::DisplayOnRightSlot(cv::Mat* img)
{
    QImage QmyImage;
    QLabel* cameraDisplay_R = this->findChild<QLabel*>("cameraDisplay1");
    cv::Mat temp = img->clone();
    //cv::resize(temp, temp, cv::Size(), 2, 2);
    cv::imwrite("temp/display.jpg", temp);
    cv::cvtColor(temp, temp, cv::COLOR_BGR2RGB);
    if (temp.channels() > 1)
    {
        QmyImage = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, QImage::Format_RGB888);
    }
    else
    {
        QmyImage = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, QImage::Format_Indexed8);
    }

    QmyImage = (QmyImage).scaled(cameraDisplay_R->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    cameraDisplay_R->setPixmap(QPixmap::fromImage(QmyImage));
}

void BLcamera::DisplayPositionSlot(double x,double y,double z,double theta)
{
    QLineEdit* lineEditX = this->findChild<QLineEdit*>("lineEditX");
    QLineEdit* lineEditY = this->findChild<QLineEdit*>("lineEditY");
    QLineEdit* lineEditZ = this->findChild<QLineEdit*>("lineEditZ");
    QLineEdit* lineEditTheta = this->findChild<QLineEdit*>("lineEditTheta");
    lineEditX->setText(QString::number(x,'f',4));
    lineEditY->setText(QString::number(y,'f',4));
    lineEditZ->setText(QString::number(z,'f',4));
    lineEditTheta->setText(QString::number(theta,'f',4));
}

int BLcamera::ClickSave2BMP()
{
    QRadioButton* radioBMP = this->findChild<QRadioButton*>("radioBMP");
    radioBMP->setChecked(true);
    m_cameraStatue.SaveImageType = MV_Image_Bmp;
    return 0;
}

int BLcamera::ClickSave2PNG()
{
    QRadioButton* radioPNG = this->findChild<QRadioButton*>("radioPNG");
    radioPNG->setChecked(true);
    m_cameraStatue.SaveImageType = MV_Image_Png;
    return 0;
}

int BLcamera::ClickSave2TIFF()
{
    QRadioButton* radioTIFF = this->findChild<QRadioButton*>("radioTIFF");
    radioTIFF->setChecked(true);
    m_cameraStatue.SaveImageType = MV_Image_Tif;
    return 0;
}

int BLcamera::ClickSave2JPG()
{
    QRadioButton* radioJPG = this->findChild<QRadioButton*>("radioJPG");
    radioJPG->setChecked(true);
    m_cameraStatue.SaveImageType = MV_Image_Jpeg;
    return 0;
}


void BLcamera::ShowErrorMsg(QString csMessage, int nErrorNum)
{
    QString errorMsg;
    if (nErrorNum == 0)
    {
        errorMsg = csMessage;
    }
    else
    {
        errorMsg = csMessage + ": Error = " + nErrorNum + ": ";
    }

    switch (nErrorNum)
    {
    case MV_E_HANDLE:           errorMsg += "Error or invalid handle ";                                         break;
    case MV_E_SUPPORT:          errorMsg += "Not supported function ";                                          break;
    case MV_E_BUFOVER:          errorMsg += "Cache is full ";                                                   break;
    case MV_E_CALLORDER:        errorMsg += "Function calling order error ";                                    break;
    case MV_E_PARAMETER:        errorMsg += "Incorrect parameter ";                                             break;
    case MV_E_RESOURCE:         errorMsg += "Applying resource failed ";                                        break;
    case MV_E_NODATA:           errorMsg += "No data ";                                                         break;
    case MV_E_PRECONDITION:     errorMsg += "Precondition error, or running environment changed ";              break;
    case MV_E_VERSION:          errorMsg += "Version mismatches ";                                              break;
    case MV_E_NOENOUGH_BUF:     errorMsg += "Insufficient memory ";                                             break;
    case MV_E_ABNORMAL_IMAGE:   errorMsg += "Abnormal image, maybe incomplete image because of lost packet ";   break;
    case MV_E_UNKNOW:           errorMsg += "Unknown error ";                                                   break;
    case MV_E_GC_GENERIC:       errorMsg += "General error ";                                                   break;
    case MV_E_GC_ACCESS:        errorMsg += "Node accessing condition error ";                                  break;
    case MV_E_ACCESS_DENIED:	errorMsg += "No permission ";                                                   break;
    case MV_E_BUSY:             errorMsg += "Device is busy, or network disconnected ";                         break;
    case MV_E_NETER:            errorMsg += "Network error ";                                                   break;
    }
    QMessageBox::warning(this, "Information", errorMsg, QMessageBox::Yes);
}