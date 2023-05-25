#include "SettingWidget.h"

SettingWidget::SettingWidget(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	ui.distortionMatrixTitle->setFixedSize(ui.distortionMatrixTitle->parentWidget()->width(),25);
	ui.internalParaMatrixTitle->setFixedSize(ui.internalParaMatrixTitle->parentWidget()->width(),25);
	ui.ExternalParaMatrixTitle->setFixedSize(ui.ExternalParaMatrixTitle->parentWidget()->width(), 25);
	ui.cameraTitle->setFixedHeight(35);
}

SettingWidget::~SettingWidget()
{
}
