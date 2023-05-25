#include "ContentWidget.h"

ContentWidget::ContentWidget(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	ui.dataDisplay->setFixedWidth(this->width()/4);
}

ContentWidget::~ContentWidget()
{
}
