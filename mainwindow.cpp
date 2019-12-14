#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <time.h>
#include <random>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
QMainWindow(parent),
ui(new Ui::MainWindow),
_frameCounter(0),
_lowerBound(0),
_upperBound(9573),
_cornerPointsSharp(),
_surfPointsFlat(),
_laserCloudFullRes() {
    _root = "/media/sukie/Ivy/dataset_20191210/dataset_0/dataset/";
    loadPointCloud();

    ui->setupUi(this);
    glviewer = new GLWidget;
    ui->scrollArea->setWidget(glviewer);

    timer = new QTimer(this);

    connect(timer, SIGNAL(timeout()), this, SLOT(draw()));
    connect(ui->pushButton_start, SIGNAL(clicked(bool)), this, SLOT(slotPlayPause()));
    connect(ui->pushButton_stop, SIGNAL(clicked(bool)), this, SLOT(slotExit()));
    connect(ui->pushButton_next, SIGNAL(clicked(bool)), this, SLOT(slotNextFrame()));
    connect(ui->pushButton_previous, SIGNAL(clicked(bool)), this, SLOT(slotPreviousFrame()));
    connect(ui->pushButton_skipForward, SIGNAL(clicked(bool)), this, SLOT(slotSkipForward()));
    connect(ui->pushButton_skipBackward, SIGNAL(clicked(bool)), this, SLOT(slotSkipBackward()));

    timer->start(50);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::draw() {
    float z_min = 999,z_max = -999;
    for (auto p : _laserCloudFullRes) {
        if (p.z < z_min) z_min = p.z;
        if (p.z > z_max) z_max = p.z;
    }

    std::vector<DrawingElem> elems;
    DrawingElem oneelem;

    // The Full-resolution PointCloud
    std::vector<point3fi> pos;
    std::vector<color3b> rgb;
    for (auto pt_: _laserCloudFullRes){
        point3fi pt;
        color3b color;
        pt.x = pt_.x ;
        pt.y = pt_.y;
        pt.z = pt_.z;

        if (true) {
            color.r = color.g = color.b = 128;
            pos.push_back(pt);
            rgb.push_back(color);
        }
    }
    oneelem.pts = pos;
    oneelem.colors = rgb;
    oneelem.pointsize = 2;
    oneelem.type = POINTS;
    elems.push_back(oneelem);

    // The Surface-feature PointCloud
    pos.clear();
    rgb.clear();
    for (auto pt_: _surfPointsFlat){
        point3fi pt;
        color3b color;
        pt.x = pt_.x ;
        pt.y = pt_.y;
        pt.z = pt_.z;

        if (true) {
            color.r = 0;
            color.g = 0;
            color.b = 255;
            pos.push_back(pt);
            rgb.push_back(color);
        }
    }
    oneelem.pts = pos;
    oneelem.colors = rgb;
    oneelem.pointsize = 6;
    oneelem.type = POINTS;
    elems.push_back(oneelem);

    // The Corner-feature PointCloud
    pos.clear();
    rgb.clear();
    for (auto pt_: _cornerPointsSharp){
        point3fi pt;
        color3b color;
        pt.x = pt_.x ;
        pt.y = pt_.y;
        pt.z = pt_.z;

        if (true) {
            color.r = 255;
            color.g = 0;
            color.b = 0;
            pos.push_back(pt);
            rgb.push_back(color);
        }
    }
    oneelem.pts = pos;
    oneelem.colors = rgb;
    oneelem.pointsize = 6;
    oneelem.type = POINTS;
    elems.push_back(oneelem);

    glviewer->setPointCloudFrame(elems);
    timer->stop();
    ui->statusBar->showMessage(QString("frame: %1, point number: %2, corner feature number: %3, flat feature number: %4, min height: %5, max height: %6")
                               .arg(QString::number(_frameCounter,10))
                               .arg(QString::number(_laserCloudFullRes.size(),10))
                               .arg(QString::number(_cornerPointsSharp.size(),10))
                               .arg(QString::number(_surfPointsFlat.size(),10))
                               .arg(QString::number(z_min,'f',3))
                               .arg(QString::number(z_max,'f',3)));
}

void MainWindow::slotPlayPause() {
    timer->start(50);
}

void MainWindow::slotExit() {
    exit(1);
}

void MainWindow::slotNextFrame() {
    _frameCounter += 10;
    drawSpecifiedFrame();
}

void MainWindow::slotPreviousFrame() {
    _frameCounter-=10;
    drawSpecifiedFrame();
}

void MainWindow::slotSkipForward() {
    _frameCounter += 100;
    drawSpecifiedFrame();
}

void MainWindow::slotSkipBackward() {
    _frameCounter -= 100;
    drawSpecifiedFrame();
}

void MainWindow::loadPointCloud() {
    QString cornerPointsSharpName = _root + QString::number(_frameCounter,10) + "/cornerPointsSharp.pcd";
    QString surfPointsFlatName = _root + QString::number(_frameCounter,10) + "/surfPointsFlat.pcd";
    QString laserCloudFullResName = _root + QString::number(_frameCounter,10) + "/laserCloudFullRes.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (cornerPointsSharpName.toStdString(), _cornerPointsSharp) == -1
     || pcl::io::loadPCDFile<pcl::PointXYZI> (surfPointsFlatName.toStdString(), _surfPointsFlat) == -1
     || pcl::io::loadPCDFile<pcl::PointXYZI> (laserCloudFullResName.toStdString(), _laserCloudFullRes) == -1) {
      PCL_ERROR ("Couldn't read PCD files\n");
      return;
    }
}

void MainWindow::drawSpecifiedFrame() {
    while (_frameCounter > _upperBound) {
        _frameCounter = _frameCounter - _upperBound + _lowerBound - 1;
    }
    while (_frameCounter < _lowerBound) {
        _frameCounter= _frameCounter - _lowerBound + _upperBound + 1;
    }
    loadPointCloud();
    timer->start(50);
}

void hsv2Rgb(double h, double s, double v, uchar &r_out, uchar &g_out, uchar &b_out)
{
    double r,g,b;
    int h_i = int(h*6);
    double f = h*6 - h_i;
    double p = v * (1 - s);
    double q = v * (1 - f*s);
    double t = v * (1 - (1 - f) * s);

    r = g = b = 0;
    switch (h_i) {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
        r = v;
        g = p;
        b = q;
        break;
    default:
        break;
    }

    r_out = uchar(r*255.0);
    g_out = uchar(g*255.0);
    b_out = uchar(b*255.0);
}

void genRandomColor(color3b & color) {
    double random_h = rand()%100/(double)101;

    double golden_ratio_conjugate = 0.618033988749895;

    random_h += golden_ratio_conjugate;
    while (random_h > 1.0)
        random_h -= 1.0;

    hsv2Rgb(random_h, 0.99, 0.99, color.r, color.g, color.b);
}
