#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <time.h>
#include <random>
#include <QDebug>
#include <iostream>
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
void genRandomColor(color3b & color)
{
    double random_h = rand()%100/(double)101;

    double golden_ratio_conjugate = 0.618033988749895;

    random_h += golden_ratio_conjugate;
    while (random_h > 1.0)
        random_h -= 1.0;

    hsv2Rgb(random_h, 0.99, 0.99, color.r, color.g, color.b);
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    std::string path = "/media/sukie/EAGET/demo_data/2";

    logobj.loadLog(path);

    glviewer = new GLViewer();

    ui->scrollArea->setWidget(glviewer);

    timer = new QTimer(this);

    connect(timer, SIGNAL(timeout()), this, SLOT(slot_timer()));

    connect(ui->pushButton_start, SIGNAL(clicked(bool)), this, SLOT(slot_btn_start()));
    connect(ui->pushButton_stop, SIGNAL(clicked(bool)), this, SLOT(slot_btn_stop()));

    glviewer->makeCurrent();
    glviewer->clearDisplayList();
    displaylistbase = glGenLists(1);

    for (int i = 0; i < 1; ++i) {
        glviewer->addDisplayList(displaylistbase + i);
    }

    Eigen::Matrix4d transform;
    transform<<1,2,3,0,
            4,5,6,0,
            1,2,3,0,
            4,5,6,1;
    std::cout<<transform<<std::endl;
    std::cout<<transform.block<3,3>(0,0)<<std::endl;

    transform.block<3,3>(0,0)=transform.block<3,3>(0,0)*Eigen::AngleAxisd(30/180.0,Eigen::Vector3d::UnitX());

    Eigen::AngleAxisd a = Eigen::AngleAxisd(30/180.0,Eigen::Vector3d::UnitX());

    std::cout<<transform<<std::endl;
    int i = 0;
    i++;

}

MainWindow::~MainWindow()
{
    glviewer->clearDisplayList();
    delete ui;
}

void MainWindow::slot_timer()
{
    std::vector<point4d> laserCloudFullRes = logobj.laserCloudFullRes();
    std::vector<point4d> lineFeaturePts = logobj.lineFeaturePts();
    std::vector<point4d> planeFeaturePts = logobj.planeFeaturePts();

    glNewList(displaylistbase, GL_COMPILE_AND_EXECUTE);

    glColor3f(0.1, 0.7, 0.1);
    glPointSize(3);
    glBegin(GL_POINTS);

    for (auto p: laserCloudFullRes) {
        glVertex3f(p.x, p.y, p.z);
    }

    glEnd();

    glEndList();

    glviewer->makeCurrent();
    glviewer->updateGL();
    timer->stop();
}

void MainWindow::slot_btn_start()
{
    timer->start(50);
}

void MainWindow::slot_btn_stop()
{
    timer->stop();
}
