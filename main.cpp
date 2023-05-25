#include "BLcamera.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    BLcamera w;
    w.show();
    return a.exec();
}
