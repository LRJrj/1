#pragma once
#include "MvCamera.h"
#include <QMutex>
#include <QSettings>
#include <QList>
#include <QStringList>
#include <QThread>
#include <QLabel>
#include <QLineEdit>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <QDebug>
#include <vector>
#include "yolov5.h"

#define MAX_DEVICE_NUM          2
#define INI_NAME                "camera.ini"
#define INTMATIRXCAM1           "IntrinsicMatrixCam1"
#define INTMATIRXCAM2           "IntrinsicMatrixCam2"
#define DISMATRIXCAM1           "DistortionMatrixCam1"
#define DISMATRIXCAM2           "DistortionMatrixCam2"
#define ROTATIONOFCAM2          "RotationMatrixOfCam2"
#define TRANSOFCAM2		        "TranslationMatrixOfCam2"
#define ROICAM1					"RoiCam1"
#define ROICAM2					"RoiCam2"


// ch:���Ⱥ궨�� | en:
#define MAX_BUF_SIZE            (1024*1024*58)
#define MAX_XML_FILE_SIZE       (1024*1024*3)

// ch:���������붨�� | en:
#define STATUS_ERROR            0

// ch:����ģʽ | en:
#define TRIGGER_OFF             0
#define TRIGGER_ON              1
#define TRIGGER_NOT_SET         -1

// ch:����Դ | en:
#define SOFTWAREMODE            7
#define HAREWAREMODE            0

#define STATUS_ERROR            -1
#define IMAGE_NAME_LEN          256

typedef struct StereoParams {
	cv::Mat IntrinsicMatrixCam1;
	cv::Mat IntrinsicMatrixCam2;
	cv::Mat RotationMatrixOfCam2;
	cv::Mat TranslationMatrixOfCam2;
	cv::Mat DistortionMatrixCam1;
	cv::Mat DistortionMatrixCam2;
	//////////////////////////////
	cv::cuda::GpuMat RectifyMapCam1[2];
	cv::cuda::GpuMat RectifyMapCam2[2];
	cv::Mat RoiCam1;
	cv::Mat RoiCam2;
}StereoParams;
typedef struct CAMERA_STATUS
{
	bool bOpenDevice;//�Ƿ��
	bool bStartGrabbing;//�Ƿ�ץͼ
	bool bMeasure;    //�Ƿ���
	int nTriggerMode;//����ģʽ
	int nTriggerSource;//����Դ
	int nSelectDeviceNum;	//ѡ���������
	int nSingleOrDouble;	//��˫���ģʽ
	/**********�������*************/
	double dExposureTime;
	double dGain;
	double dFrameRate;
	/**********�����������*************/
	StereoParams stereoParams;

	MV_SAVE_IAMGE_TYPE   SaveImageType;	//����ͼ���ʽ
}CAMERA_STATUS;								//���״̬

typedef struct MYCAMERA
{
	CMvCamera* pcMyCamera;					//�������
	QMutex SaveImageMux;					//��
	cv::Mat* myImage;						//�������ͼ��ָ��
	cv::cuda::GpuMat ImageCache;			//GPU����ͼƬ����
}MYCAMERA;

typedef struct IniData {
	QString key;
	QString value;
	IniData(QString keyName,QString valueName) :key(""), value("")
	{
		key = keyName;
		value = valueName;
	}
};

typedef struct           // Ŀ����
{
	cv::Rect MonitorArea;				// �������
	//cvScalar MeanThreshold;         // ��ֵ��ֵ
	//cvScalar StdDevThreshold;       // ��׼����ֵ
	cv::cuda::GpuMat BackGroundImage;		// ����ģ��
}TargetMonitor;

typedef struct           // ����λ��
{
	double    x;				// x
	double    y;				// y
	double    z;				// z
	double    agl;				// deg
}Position;

typedef struct           // ��������ͽǶ�
{
	cv::Point2d cpt;				// ��������
	double    agl;				// ��ת�Ƕ�
}CptAgl;

//��������
void WriteIniFile(QString iniName, QString group, QList<IniData> iniData);
void ReadIniFile(QString iniName, QString group, QList<IniData>& iniData);
QVariant ReadIniFile(QString iniName, QString group, QString key);

//ץͼ�߳�
class GrabThread : public QThread
{
	Q_OBJECT

public:
	GrabThread();
	~GrabThread();
	void run();
	void getCameraPtr(CMvCamera* camera);
	void getImagePtr(cv::Mat* image);
	void getCameraIndex(int index);
	void getDisplay(QLabel* label);
	void getMeasureFlag(bool flag);
	void getLineFlag(bool flag);
	void getCameraImageCache(cv::cuda::GpuMat* imageCache);
	void getSaveImageMux(QMutex* mutex);
	void Display();
	void DisplayFrameCount();

	//Slots
	void TargetFlagSlot(bool bTargetFlag);
signals:
	void Display(const cv::Mat* image, int index);
	void DisplayImage(QImage image);
	void DislpayFrame(int recFrame, int lostFrame);
private:
	CMvCamera* cameraPtr = NULL;
	cv::Mat* imagePtr = NULL;
	QLabel* displayLabel = NULL;
	cv::cuda::GpuMat* imageCache;
	QMutex* SaveImageMux;
	int cameraIndex = NULL;
	int TriggerMode;
	bool bMeasure = false;
	bool bTargetFlag = false;
	bool bLineDisplay = false;
};

class DetectThread :public QThread
{
	Q_OBJECT

public:
	DetectThread();
	~DetectThread();
	void run();
	void InitMonitor(int nMode);
	bool TargetStdDev(cv::cuda::GpuMat curImg, TargetMonitor tm);
	void TargetDetection();
	void PositionAndPosture();
	Position CalPositionAndPosture(cv::cuda::GpuMat& img1, cv::cuda::GpuMat& img2, StereoParams sp);
	CptAgl FindContours(cv::cuda::GpuMat gImg, int id, cv::cuda::GpuMat gMapx, cv::cuda::GpuMat gMapy, cv::Mat Roi);
	void getSaveImageMux(QMutex* mutexR, QMutex* mutexL);
	void getCameraImageCache(cv::cuda::GpuMat* imageCacheR, cv::cuda::GpuMat* imageCacheL);
	void getSps(StereoParams sp);
signals:
	void TargetFlagSignal(bool flag);
	void DisplaySignal(cv::Mat* image);
	void PosSignal(double, double, double, double);
public:
	bool bTargetInFlag = false;
	bool bTargetOutFlag = false;
	QMutex* SaveImageMuxR;
	QMutex* SaveImageMuxL;
	cv::cuda::GpuMat* imageCache_R;
	cv::cuda::GpuMat* imageCache_L;
	TargetMonitor moniter = {  };
	StereoParams cameraSP;
	Yolov5* yolo;
};
