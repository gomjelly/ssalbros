#include "ssalbros.h"
#include <QtWidgets/QApplication>
//#include <opencv2/opencv.hpp>
//#include <iostream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ssalbros w;
    w.show();
    return a.exec();
}
