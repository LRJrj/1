#pragma once

#include <QWidget>
#include "ui_TitleWidget.h"

class TitleWidget : public QWidget
{
	Q_OBJECT

public:
	TitleWidget(QWidget *parent = Q_NULLPTR);
	~TitleWidget();

private:
	Ui::TitleWidget ui;
};
