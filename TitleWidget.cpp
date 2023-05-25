#include "TitleWidget.h"
#include "BLcamera.h"

TitleWidget::TitleWidget(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	QImage image;
	image.load(":/image/nimte.png");
	ui.iconLabel->resize(image.width() * 0.5, image.height() * 0.5);
	image = image.scaled(image.width() * 0.5, image.height() * 0.5);
	ui.iconLabel->setPixmap(QPixmap::fromImage(image));
	ui.iconLabel->move(this->x() , this->y() );
}

TitleWidget::~TitleWidget()
{
}
