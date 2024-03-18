#include "ssalbros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <QThread>
#include <QImage>
#include <QPainter>

ssalbros::ssalbros(QWidget *parent)
    : QMainWindow(parent)
{
    // Qt ���� ����
    m_scene = new QGraphicsScene();
    m_view = new QGraphicsView(m_scene);
    m_view->setWindowTitle("Camera View");
    m_view->show();

    this->setCentralWidget(m_view);
    this->startRender();

    this->setFixedSize(800, 600);
}

ssalbros::~ssalbros()
{}

void ssalbros::startRender()
{
    // ī�޶� ������ �����忡�� ���� ���� �غ�
    QThread* thread = new QThread;
    CameraWorker* worker = new CameraWorker();
    worker->moveToThread(thread);

    connect(thread, &QThread::started, worker, &CameraWorker::process);
    connect(worker, &CameraWorker::finished, thread, &QThread::quit);
    connect(worker, &CameraWorker::finished, worker, &CameraWorker::deleteLater);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);

    // ī�޶� �������� �޾� ������Ʈ�ϴ� �ñ׳�
    connect(worker, &CameraWorker::frameReady, this, [=](const QImage& img) {
        QPixmap pixmap = QPixmap::fromImage(img);
        m_scene->clear();
        m_scene->addPixmap(pixmap);
        });

    thread->start();
}

void drawPose(cv::Mat& frame, const cv::Mat& points) {
    int nPairs = 20; // COCO dataset������ ���� �� ����
    int posePairs[20][2] = {
        {1, 2}, {1, 5}, {2, 3}, {3, 4}, {5, 6}, {6, 7},  // ��
        {1, 8}, {8, 9}, {9, 10}, {1, 11}, {11, 12}, {12, 13}, // ����
        {1, 0}, {0, 14}, {14, 16}, {0, 15}, {15, 17}  // �󱼰� �ٸ�
    };

    for (int i = 0; i < nPairs; i++) {
        // ���� ���� ��ǥ�� �����ɴϴ�.
        cv::Point partA = cv::Point((int)points.at<float>(0, posePairs[i][0]), (int)points.at<float>(1, posePairs[i][0]));
        cv::Point partB = cv::Point((int)points.at<float>(0, posePairs[i][1]), (int)points.at<float>(1, posePairs[i][1]));

        if (partA.x <= 0 || partA.y <= 0 || partB.x <= 0 || partB.y <= 0)
            continue;

        line(frame, partA, partB, cv::Scalar(0, 255, 255), 8);
        circle(frame, partA, 8, cv::Scalar(0, 0, 255), -1);
        circle(frame, partB, 8, cv::Scalar(0, 0, 255), -1);
    }
}

// CameraWorker Ŭ���� ����
void CameraWorker::process() {
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        emit finished();
        return;
    }

    // �� ���� �� ���� ���� ���
    std::string modelFile = "D:\\MyProject\\pose_iter_440000.caffemodel";
    std::string configFile = "D:\\MyProject\\pose_deploy_linevec.prototxt";

    cv::Mat frame;
    cv::Ptr<cv::dnn::Net> net = cv::makePtr<cv::dnn::Net>(cv::dnn::readNetFromCaffe(configFile, modelFile));

    while (true) {
        cv::Mat frame;
        cap.read(frame);

        if (frame.empty()) break;
        cv::flip(frame, frame, 1); // frame�� �¿�� ����

        //
        cv::Mat inputBlob = cv::dnn::blobFromImage(frame, 1.0 / 255, cv::Size(368, 368), cv::Scalar(0, 0, 0), false, false);
        net->setInput(inputBlob);
        cv::Mat output = net->forward();

        // ��� ��Ʈ������ ������ ����ϴ�.
        int H = output.size[2];
        int W = output.size[3];

        // �� ������ ��ġ�� ������ ��Ʈ������ �����մϴ�.
        cv::Mat points(2, 18, CV_32F, cv::Scalar(0));

        for (int n = 0; n < 18; n++) {
            // �ش� ��ü ������ ���� ��Ʈ�ʿ��� �ִ밪�� ã���ϴ�.
            cv::Mat probMap(H, W, CV_32F, output.ptr(0, n));
            cv::Point maxLoc;
            double prob;
            minMaxLoc(probMap, 0, &prob, 0, &maxLoc);

            if (prob > 0.1) {
                points.at<float>(0, n) = (float)(maxLoc.x) * frame.cols / W;
                points.at<float>(1, n) = (float)(maxLoc.y) * frame.rows / H;
            }
        }

        drawPose(frame, points);

        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB); // BGR���� RGB�� ��ȯ
        QImage img(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        img.bits(); // QImage�� �����͸� �����ϵ��� ����

        emit frameReady(img);
        QThread::msleep(33); // �� 30������ �ӵ�
    }

    emit finished();
}