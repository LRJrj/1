#pragma once

#include <QWidget>
#include "ui_ContentWidget.h"

class ContentWidget : public QWidget
{
	Q_OBJECT

public:
	ContentWidget(QWidget *parent = Q_NULLPTR);
	~ContentWidget();

public:
	Ui::ContentWidget ui;
};
