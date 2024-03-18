#pragma once

#include <QtWidgets/QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QTimer>
#include <QImage>
#include "ui_ssalbros.h"
class CameraWorker : public QObject
{
    Q_OBJECT
public:
    CameraWorker(){};
    ~CameraWorker(){};

public slots:
    void process();

signals:
    void finished();
    void frameReady(const QImage& img);

};

class ssalbros : public QMainWindow
{
    Q_OBJECT

public:
    ssalbros(QWidget *parent = nullptr);
    ~ssalbros();

    void startRender();

private:
    QGraphicsScene* m_scene;
    QGraphicsView* m_view;
};
