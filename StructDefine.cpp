#include "StructDefine.h"
#include <QDebug>
#include <QMutex>
#define Fan 0
#define GWJ 1
QMutex mutex;

void WriteIniFile(QString iniName, QString group, QList<IniData> iniData)
{
	QSettings settings(iniName, QSettings::IniFormat);
	settings.beginWriteArray(group);
	//settings.clear();
	for (int i = 0; i < iniData.size(); i++)
	{
		//settings.setArrayIndex(i);
	    settings.setValue(iniData.at(i).key, iniData.at(i).value);
	}
	settings.endArray();
}

void ReadIniFile(QString iniName, QString group, QList<IniData>& iniData)
{
	if (!iniData.empty())
	{
		iniData.clear();
	}
	QSettings settings(iniName, QSettings::IniFormat);
	settings.beginReadArray(group);
	QStringList list = settings.childKeys();
	for (int i = 0; i < list.size()-1; i++)//减1去除一个size选项
	{
		//settings.setArrayIndex(i);
		QString key = list.at(i);
		QString value = (settings.value(key)).toString();
		IniData temp(key,value);
		iniData.append(temp);
	}
	settings.endArray();
}

QVariant ReadIniFile(QString iniName, QString group, QString key)
{
	QSettings settings(iniName, QSettings::IniFormat);
	settings.beginGroup(group);
	return QVariant(settings.value(key));
}


GrabThread::GrabThread()
{
}

GrabThread::~GrabThread()
{
	terminate();
	if (cameraPtr != NULL)
	{
		delete cameraPtr;
	}
	if (imagePtr != NULL)
	{
		delete imagePtr;
	}
}
void GrabThread::getCameraPtr(CMvCamera* camera)
{
	cameraPtr = camera;
}
void GrabThread::getImagePtr(cv::Mat* image)
{
	imagePtr = image;
}
void GrabThread::getCameraIndex(int index)
{
	cameraIndex = index;
}
void GrabThread::getDisplay(QLabel* label)
{
	displayLabel = label;
}
void GrabThread::getMeasureFlag(bool flag)
{
	bMeasure = flag;
}
void GrabThread::getLineFlag(bool flag)
{
	bLineDisplay = flag;
}
void GrabThread::getCameraImageCache(cv::cuda::GpuMat* imageCache)
{
	this->imageCache = imageCache;
}
void GrabThread::getSaveImageMux(QMutex* mutex)
{
	this->SaveImageMux = mutex;
}
void GrabThread::Display()
{
	cv::Mat rgb;
	cv::cvtColor(*imagePtr, rgb, CV_BGR2RGB);

	if (bLineDisplay)
	{
		cv::Point p0 = cv::Point(20, rgb.rows / 2);
		cv::Point p1 = cv::Point(rgb.cols-200, rgb.rows / 2);
		cv::line(rgb, p0, p1, cv::Scalar(255, 0, 0), 4, 8);
		
		/*cv::imwrite("temp/11.jpg", rgb);
		cv::Rect b = cv::Rect(300, 250, 900, 100);
		cv::Mat a = rgb(b);
		cv::imwrite("temp/22.jpg", a);*/
	}
	//判断是黑白、彩色图像
	QImage QmyImage;
	if (imagePtr->channels() > 1)
	{
		QmyImage = QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, QImage::Format_RGB888);
	}
	else
	{
		QmyImage = QImage((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, QImage::Format_Indexed8);
	}

	//QmyImage = (QmyImage).scaled(displayLabel->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	//显示图像
	//displayLabel->setPixmap(QPixmap::fromImage(QmyImage));
	emit DisplayImage(QmyImage);
	if (cameraIndex == 1)
	{
		qDebug() << "main 1 camera Grabbing";
	}
	if (cameraIndex == 0)
	{
		qDebug() << "main 0 camera Grabbing";
	}
}
void GrabThread::DisplayFrameCount()
{
	int nFrameCount = 0;
	int nLostFrame = 0;
	MV_MATCH_INFO_NET_DETECT stMatchInfoNetDetect = { 0 };
	MV_MATCH_INFO_USB_DETECT stMatchInfoUSBDetect = { 0 };
	MV_CC_DEVICE_INFO stDevInfo = { 0 };
	cameraPtr->GetDeviceInfo(&stDevInfo);
	if (stDevInfo.nTLayerType == MV_GIGE_DEVICE)
	{
		cameraPtr->GetGevAllMatchInfo(&stMatchInfoNetDetect);
		nLostFrame = stMatchInfoNetDetect.nLostFrameCount;
		nFrameCount = stMatchInfoNetDetect.nNetRecvFrameCount;
	}
	else if (stDevInfo.nTLayerType == MV_USB_DEVICE)
	{
		cameraPtr->GetU3VAllMatchInfo(&stMatchInfoUSBDetect);
		nLostFrame = stMatchInfoUSBDetect.nErrorFrameCount;
		nFrameCount = stMatchInfoUSBDetect.nReceivedFrameCount;
	}
	emit DislpayFrame(nFrameCount % 1000000, nLostFrame % 1000000);
}
void GrabThread::TargetFlagSlot(bool bTargetFlag)
{
	this->bTargetFlag = bTargetFlag;
	qDebug() << "rec TargetFlag" << bTargetFlag;
}
void GrabThread::run()
{
	if (cameraPtr == NULL) {
		return;
	}

	if (imagePtr == NULL) {
		return;
	}
	//int i = 0;
	while (!isInterruptionRequested())
	{
		cameraPtr->ReadBuffer(*imagePtr);
		if (!bMeasure)
		{
			Display();
		}
		//if(bMeasure||bTargetFlag)
		if (true)
		{
			cv::Mat rgbImg;
			SaveImageMux->lock();
			
			if (!(this->imageCache->empty()))
			{
				this->imageCache->release();
			}
			//cv::cvtColor(*imagePtr, grey, CV_BGR2GRAY);//彩色转为黑白上传保存
			/*QString str;
			str.sprintf("temp/%d.jpg", i);
			cv::imwrite(str.toStdString(), grey);
			i++;*/
			//彩色原图上传保存
			rgbImg = imagePtr->clone();
			this->imageCache->upload(rgbImg);  //CPU Mat upload to GPU Mat
			SaveImageMux->unlock();
			if (bMeasure)
			{
				qDebug() << "right";
			}
			if (bTargetFlag)
			{
				qDebug() << "left";
			}
		}
		DisplayFrameCount();
	}
}

DetectThread::~DetectThread()
{
	if (yolo != NULL)
	{
		delete yolo;
	}
}
DetectThread::DetectThread()
{
	yolo = new Yolov5();
	yolo->Initialize("myyolov5s.engine", 0);
}

void DetectThread::run()
{
	
	int i = 0;
	while (!isInterruptionRequested())
	{
		TargetDetection();
		if (bTargetInFlag && bTargetOutFlag)
		{
			//PositionAndPosture();
		}
		//sleep(10);
	}
}

void DetectThread::InitMonitor(int nMode)
{
	if (0 == nMode)
	{
		if (!moniter.BackGroundImage.empty())
		{
			moniter.BackGroundImage.release();
		}
		return;
	}
	moniter.MonitorArea = cv::Rect(200, 100, 900, 100);
	//moniter.MonitorArea = cv::Rect();
	while (moniter.BackGroundImage.empty())
	{
		SaveImageMuxR->lock();
		
		if (!(this->imageCache_R->empty()))
		{
			cv::cuda::GpuMat gray;
			if (imageCache_R->channels() > 1)
			{
				cv::cuda::cvtColor(*imageCache_R, gray, CV_BGR2GRAY);
			}
			else
			{
				gray = imageCache_R->clone();
			}
			moniter.BackGroundImage = gray(moniter.MonitorArea).clone();
		}
		SaveImageMuxR->unlock();
	}
}

bool DetectThread::TargetStdDev(cv::cuda::GpuMat curImg, TargetMonitor tm)
{
	cv::Scalar Mean, StdDev;
	curImg = curImg(tm.MonitorArea);
	cv::cuda::absdiff(curImg,tm.BackGroundImage,curImg);
	cv::cuda::meanStdDev(curImg,Mean,StdDev);
	if (StdDev.val[0] > 20)
	{
		return true;
	}
	return false;
}

void DetectThread::TargetDetection()
{
	SaveImageMuxR->lock();
	cv::cuda::GpuMat TempMat;

	if (imageCache_R->empty())
	{
		SaveImageMuxR->unlock();
		return;
	}

	if (imageCache_R->channels() > 1)
	{
		cv::cuda::cvtColor(*imageCache_R, TempMat, CV_BGR2GRAY);
	}
	else
	{
		TempMat = imageCache_R->clone();
	}
	//imageCache_R->release();
	SaveImageMuxR->unlock();
	
	bool bRet = TargetStdDev(TempMat, moniter);

	//上升沿检测
	if ((!bTargetInFlag) && bRet)
	{
		bTargetInFlag = true;
	}
	//下降沿检测
	if (bTargetInFlag && (!bRet))
	{
		bTargetOutFlag = true;
	}
	//切换到定位计算
	if (bTargetInFlag && bTargetOutFlag)
	{
		//qDebug() << "send signal";
		emit TargetFlagSignal(true);
	}
}

void DetectThread::PositionAndPosture()
{
	//无yolo识别的定位
#if 0
	cv::cuda::GpuMat gImg1, gImg2;
	SaveImageMuxL->lock();
	SaveImageMuxR->lock();
	if (imageCache_R->empty() || imageCache_L->empty())
	{
		SaveImageMuxR->unlock();
		SaveImageMuxL->unlock();
		return;
	}
	if (imageCache_L->channels() > 1)
	{
		cv::cuda::cvtColor(*imageCache_L, gImg1, CV_BGR2GRAY);
	}
	else
	{
		gImg1 = imageCache_L->clone();
	}
	if (imageCache_R->channels() > 1)
	{
		cv::cuda::cvtColor(*imageCache_R, gImg2, CV_BGR2GRAY);
	}
	else
	{
		gImg2 = imageCache_R->clone();
	}
	SaveImageMuxL->unlock();
	SaveImageMuxR->unlock();

	Position pos = { 0 };
	if (!(gImg1.empty() || gImg2.empty()))
	{
		pos = CalPositionAndPosture(gImg1, gImg2, cameraSP);
		emit PosSignal(pos.x,pos.y,pos.z,pos.agl);
	}
	bTargetInFlag = false;
	bTargetOutFlag = false;
#endif // 0
	//加入yolo的定位
#if 1
	cv::cuda::GpuMat gImg1, gImg2;
	SaveImageMuxL->lock();
	SaveImageMuxR->lock();
	if (imageCache_R->empty() || imageCache_L->empty())
	{
		SaveImageMuxR->unlock();
		SaveImageMuxL->unlock();
		return;
	}
	gImg1 = imageCache_L->clone();
	gImg2 = imageCache_R->clone();
	SaveImageMuxL->unlock();
	SaveImageMuxR->unlock();

	Position pos = { 0 };
	if (!(gImg1.empty() || gImg2.empty()))
	{
		pos = CalPositionAndPosture(gImg1, gImg2, cameraSP);
		emit PosSignal(pos.x, pos.y, pos.z, pos.agl);
	}
	bTargetInFlag = false;
	bTargetOutFlag = false;
#endif // 1

}

Position DetectThread::CalPositionAndPosture(cv::cuda::GpuMat& img1, cv::cuda::GpuMat& img2, StereoParams sp)
{
	Position pos = { 0 };
	CptAgl calImg1 = { 0 }, calImg2 = { 0 };
	calImg1 = FindContours(img1, 0, sp.RectifyMapCam1[0], sp.RectifyMapCam1[1], sp.RoiCam1);
	calImg2 = FindContours(img2, 1, sp.RectifyMapCam2[0], sp.RectifyMapCam2[1], sp.RoiCam2);

	cv::Rect2d range(-1, -1, 1, 1);
	//计算位姿
	if (!(calImg1.cpt.inside(range) || calImg2.cpt.inside(range)))
	{
		//主相机
		double dPosXInCamL, dPosYInCamL, dPosZInCamL, dAglInCamL;
		double dFocusXInCamL = sp.IntrinsicMatrixCam1.at<double>(0, 0);
		double dFocusYInCamL = sp.IntrinsicMatrixCam1.at<double>(1, 1);
		double dFocusInCamL = (dFocusXInCamL + dFocusYInCamL) / 2;
		// 以辅相机计算 
		double dPosXInCamR, dPosYInCamR, dPosZInCamR, dAglInCamR;
		double dFocusXInCamR = sp.IntrinsicMatrixCam2.at<double>(0, 0);
		double dFocusYInCamR = sp.IntrinsicMatrixCam2.at<double>(1, 1);
		double dFocusInCamR = (dFocusXInCamR + dFocusYInCamR) / 2;

		double dBaseLine = sqrt(pow(sp.TranslationMatrixOfCam2.at<double>(0), 2)
			+ pow(sp.TranslationMatrixOfCam2.at<double>(1), 2)
			+ pow(sp.TranslationMatrixOfCam2.at<double>(2), 2));//基线长度dBaseLine

		dPosZInCamL = dFocusInCamL * dBaseLine / (calImg1.cpt.x - calImg2.cpt.x);
		dPosXInCamL = calImg1.cpt.x * dPosZInCamL / dFocusXInCamL;
		dPosYInCamL = calImg1.cpt.y * dPosZInCamL / dFocusYInCamL;
		dAglInCamL = calImg1.agl;

		dPosZInCamR = dFocusInCamR * dBaseLine / (calImg1.cpt.x - calImg2.cpt.x);
		dPosXInCamR = dBaseLine + calImg2.cpt.x * dPosZInCamR / dFocusXInCamR;
		dPosYInCamR = calImg2.cpt.y * dPosZInCamR / dFocusYInCamR;
		dAglInCamR = calImg2.agl;

		pos.x = abs((dPosXInCamL + dPosXInCamR) / 2);
		pos.y = abs((dPosYInCamL + dPosYInCamR) / 2);
		pos.z = abs((dPosZInCamL + dPosZInCamR) / 2);
		pos.agl = abs((dAglInCamL + dAglInCamR) / 2);
	}
	return pos;
}

CptAgl DetectThread::FindContours(cv::cuda::GpuMat gImg, int id, cv::cuda::GpuMat gMapx, cv::cuda::GpuMat gMapy, cv::Mat Roi)
{
	cv::Mat a;
	gImg.download(a);
	if (0 == id)
	{
		cv::imwrite("temp/yuantu0.jpg", a);
	}
	else
	{
		cv::imwrite("temp/yuantu1.jpg", a);
	}
	cv::Rect monitor = cv::Rect(200, 100, 900, 100);
	if (!(a.channels() > 1))
	{
		cv::cvtColor(a, a, CV_GRAY2BGR);
	}
	cv::line(a, cv::Point(monitor.x,monitor.y), cv::Point(monitor.x+monitor.width,monitor.y), cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
	cv::line(a, cv::Point(monitor.x, monitor.y + monitor.height), cv::Point(monitor.x + monitor.width, monitor.y + monitor.height), cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
	cv::line(a, cv::Point(monitor.x, monitor.y), cv::Point(monitor.x , monitor.y + monitor.height), cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
	cv::line(a, cv::Point(monitor.x+monitor.width, monitor.y ), cv::Point(monitor.x + monitor.width, monitor.y+ monitor.height), cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
	if (0 == id)
	{
		cv::imwrite("temp/monitor0.jpg", a);
	}
	else
	{
		cv::imwrite("temp/monitor1.jpg", a);
	}
	CptAgl cvCptAgl = { 0 };
#if GWJ
	cv::cuda::GpuMat gRemap;
	cv::Mat PreImage, bgImg;
	cv::cuda::remap(gImg, gRemap, gMapx, gMapy, cv::INTER_LINEAR);
	gRemap.download(PreImage);
	//加yolo
	std::vector<cv::Rect> Boxes;
	std::vector<const char*> ClassLables;
	yolo->Detecting(PreImage, Boxes, ClassLables);
	bgImg = PreImage.clone();
	if (bgImg.channels() < 3)
	{
		cv::cvtColor(bgImg, bgImg, cv::COLOR_GRAY2BGR);
	}
	if (0 == id)
	{
		cv::imwrite("temp/Remap0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/Remap1.jpg", PreImage);
	}
	//高斯滤波
	if (gRemap.channels() > 1)
	{
		cv::cuda::cvtColor(gRemap, gRemap, cv::COLOR_BGR2GRAY);
	}
	cv::cuda::GpuMat gGauss;
	cv::Ptr<cv::cuda::Filter> pGaussianFilter;
	pGaussianFilter = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(5, 5), 1.5);
	pGaussianFilter->apply(gRemap, gGauss);
	gGauss.download(PreImage);
	if (0 == id)
	{
		cv::imwrite("temp/Gauss0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/Gauss1.jpg", PreImage);
	}
	//获取ROI区域
	//cv::Rect2d recRoi(Roi.at<double>(0), Roi.at<double>(1), Roi.at<double>(2), Roi.at<double>(3));//无yolo
	//有yolo
	
	cv::Rect2d recRoi(Boxes[0].x,Boxes[0].y,Boxes[0].width,Boxes[0].height);
	cv::cuda::GpuMat contourRoi;
	contourRoi = gGauss(recRoi);
	contourRoi.download(PreImage);
	if (0 == id)
	{
		cv::imwrite("temp/roi0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/roi1.jpg", PreImage);
	}
	//二值化
	cv::cuda::GpuMat binaryImg;
	cv::Mat mbinaryImg;
	cv::Mat c;
	contourRoi.download(c);
	cv::threshold(c, mbinaryImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	PreImage = mbinaryImg.clone();
	binaryImg.upload(mbinaryImg);
	//cv::cuda::threshold(contourRoi, binaryImg, 0, 255, THRESH_BINARY | THRESH_OTSU);
	//binaryImg.download(PreImage);
	if (0 == id)
	{
		cv::imwrite("temp/binary0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/binary1.jpg", PreImage);
	}
	//开运算
	cv::cuda::GpuMat pOpen;
	cv::Mat kernel = cv::Mat::ones(cv::Size(5, 5), CV_8UC1);
	cv::Ptr<cv::cuda::Filter> pMorphologyFilter;
	pMorphologyFilter = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_8UC1, kernel, cv::Point(-1, -1), 3);
	pMorphologyFilter->apply(binaryImg, pOpen);
	pOpen.download(PreImage);
	if (0 == id)
	{
		cv::imwrite("temp/open0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/open1.jpg", PreImage);
	}
	//找角点
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(PreImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(recRoi.x, recRoi.y));
	std::vector<float> contoursArea;
	for (int i=0; i < contours.size(); i++)
	{
		contoursArea.push_back(cv::contourArea(contours[i]));
	}
	std::vector<float>::iterator biggest = std::max_element(std::begin(contoursArea), std::end(contoursArea));
	for (int i = 0; i < contours.size(); i++)
	{
		float x = cv::contourArea(contours[i]) - *biggest;
		if ((x>=-0.00001)&&(x<=0.00001))
		{
			cv::RotatedRect rrect = cv::minAreaRect(contours[i]);
			cvCptAgl.cpt = rrect.center;
			cvCptAgl.agl = rrect.angle;
			//绘制最小举证轮廓
			cv::Point2f points[4];
			rrect.points(points);
			for (int i = 0; i < 4; i++)
			{
				cv::line(bgImg, points[i % 4], points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
			}
			cv::circle(bgImg, cvCptAgl.cpt, 10, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
			break;
		}
		else
		{
			cvCptAgl = CptAgl{ 0 };
		}
	}
	if (1 == id)
	{
		//bgImg = bgImg(recRoi);
		cv::imwrite("temp/Drawcenter1.jpg", bgImg);
		emit DisplaySignal(&bgImg);
		cv::namedWindow("input", cv::WINDOW_AUTOSIZE);
		cv::imshow("input", bgImg);
		cv::waitKey(0);
		qDebug() << "fasong";
	}
	else
	{
		cv::imwrite("temp/Drawcenter0.jpg", bgImg);
	}

#endif // 

//范总检测方法
#if Fan
	cv::cuda::GpuMat gResize;
	cv::Mat PreImage;
	//缩小图片
	cv::cuda::resize(gImg, gResize, cv::Size(), 1, 1);
	gResize.download(PreImage);
	if (0 == id)
	{
		cv::imwrite("temp/afterResize0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/afterResize1.jpg", PreImage);
	}
	

	//图形矫正
	cv::cuda::GpuMat gRemap;
	cv::cuda::remap(gResize, gRemap, gMapx, gMapy, cv::INTER_LINEAR);
	cv::Mat bgImg, tempImg;
	gRemap.download(tempImg);
	std::vector<cv::Rect> Boxes;
	std::vector<const char*> ClassLables;
	yolo->Detecting(tempImg, Boxes, ClassLables);
	bgImg = tempImg.clone();
	if (1 == bgImg.channels())
	{
		cv::cvtColor(bgImg, bgImg, cv::COLOR_GRAY2BGR);
		if (0 == id)
		{
			cv::imwrite("temp/afterRemap0.jpg", bgImg);
		}
		else
		{
			cv::imwrite("temp/afterRemap1.jpg", bgImg);
		}
		
	}
	if (1 != gRemap.channels())
	{
		cv::cuda::cvtColor(gRemap, gRemap, cv::COLOR_BGR2GRAY);
	}
	//高斯滤波
	if (gRemap.channels() > 1)
	{
		cv::cuda::cvtColor(gRemap, gRemap, cv::COLOR_BGR2GRAY);
	}
	cv::cuda::GpuMat gGauss;
	cv::Ptr<cv::cuda::Filter> pGaussianFilter;
	pGaussianFilter = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(5, 5), 3);
	pGaussianFilter->apply(gRemap, gGauss);
	gGauss.download(PreImage);
	if (0 == id)
	{
		cv::imwrite("temp/afterGauss0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/afterGauss1.jpg", PreImage);
	}

	//边缘检测
	cv::cuda::GpuMat gCanny;
	cv::Ptr<cv::cuda::CannyEdgeDetector> pCannyDetector;
	pCannyDetector = cv::cuda::createCannyEdgeDetector(70, 180, 3, false);
	pCannyDetector->detect(gGauss, gCanny);
	gCanny.download(PreImage);
	if (0 == id)
	{
		cv::imwrite("temp/afterCanny0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/afterCanny1.jpg", PreImage);
	}

	//形态学 膨胀
	cv::cuda::GpuMat gDilate;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Ptr<cv::cuda::Filter> pMorphologyFilter;
	pMorphologyFilter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel);
	pMorphologyFilter->apply(gCanny, gDilate);
	
	gDilate.download(PreImage);
	if (0 == id)
	{
		cv::imwrite("temp/afterpengzhang0.jpg", PreImage);
	}
	else
	{
		cv::imwrite("temp/afterpengzhang1.jpg", PreImage);
	}

	//ROI区域
	//cv::Rect2d recRoi(Roi.at<double>(0), Roi.at<double>(1), Roi.at<double>(2), Roi.at<double>(3));
	cv::Rect2d recRoi(Boxes[0].x,Boxes[0].y,Boxes[0].width,Boxes[0].height);
	//cv::Rect2d recRoi(0, 0, 640, 512);
	cv::Mat contourRoi = PreImage(recRoi);
	if (0 == id)
	{
		cv::imwrite("temp/afterroi0.jpg", contourRoi);
	}
	else
	{
		cv::imwrite("temp/afterroi1.jpg", contourRoi);
	}
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(contourRoi, contours, 0, cv::CHAIN_APPROX_SIMPLE, cv::Point(recRoi.x, recRoi.y));
#endif // 0

#if 0
	//绘制所有脚点
	cv::Mat temp1;
	temp1 = bgImg.clone();
	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].size(); j++)
		{
			cv::circle(temp1, contours[i][j], 1, cv::Scalar(0, 255, 0), -1);
		}
	}
	if (0 == id)
	{
		cv::imwrite("temp/drawCon0.jpg", temp1);
	}
	else
	{
		cv::imwrite("temp/drawCon1.jpg", temp1);
	}
#endif
#if 0
	//绘制ROI区域
	cv::Mat DrawRoi;
	DrawRoi = bgImg.clone();
	cv::Point p1 = cv::Point(recRoi.x, recRoi.y);
	cv::Point p2 = cv::Point(recRoi.x + recRoi.width, recRoi.y);
	cv::Point p3 = cv::Point(recRoi.x, recRoi.y+ recRoi.height);
	cv::Point p4 = cv::Point(recRoi.x + recRoi.width, recRoi.y+ recRoi.height);

	cv::line(DrawRoi, p1,p2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
	cv::line(DrawRoi, p3,p4, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
	cv::line(DrawRoi, p1,p3, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
	cv::line(DrawRoi, p2,p4, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
	
	if (0==id)
	{
		cv::imwrite("temp/ROI0.jpg", DrawRoi);
	}
	else 
	{
		cv::imwrite("temp/ROI1.jpg", DrawRoi);
	}
	
#endif

#if Fan
	//轮廓查找
	for (int t = 0; t < contours.size(); t++)
	{
		cv::RotatedRect rrect = cv::minAreaRect(contours[t]);
		cvCptAgl.cpt = rrect.center;
		if (cvCptAgl.cpt.inside(recRoi))
		{
			cvCptAgl.cpt = rrect.center;
			cvCptAgl.agl = rrect.angle;
			//绘制最小举证轮廓
			cv::Point2f points[4];
			rrect.points(points);
			for (int i = 0; i < 4; i++)
			{
				cv::line(bgImg, points[i % 4], points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
			}
			cv::circle(bgImg, cvCptAgl.cpt, 10, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0);
			break;
		}
		else
		{
			cvCptAgl = CptAgl{ 0 };
		}
	}
	if (1 == id)
	{
		//bgImg = bgImg(recRoi);
		cv::imwrite("temp/Drawcenter1.jpg", bgImg);
		emit DisplaySignal(&bgImg);
		qDebug() << "fasong";
	}
	else
	{
		cv::imwrite("temp/Drawcenter0.jpg", bgImg);
	}
#endif // Fan

	return CptAgl(cvCptAgl);
}

void DetectThread::getSaveImageMux(QMutex* mutexR, QMutex* mutexL)
{
	SaveImageMuxR = mutexR;
	SaveImageMuxL = mutexL;
}

void DetectThread::getCameraImageCache(cv::cuda::GpuMat* imageCacheR, cv::cuda::GpuMat* imageCacheL)
{
	imageCache_R = imageCacheR;
	imageCache_L = imageCacheL;
}

void DetectThread::getSps(StereoParams sp)
{
	cameraSP = sp;
}
