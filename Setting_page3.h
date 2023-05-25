#pragma once

#include <QWidget>
#include "ui_Setting_page3.h"

class Setting_page3 : public QWidget
{
	Q_OBJECT

public:
	Setting_page3(QWidget *parent = Q_NULLPTR);
	~Setting_page3();

private:
	Ui::Setting_page3 ui;
};
