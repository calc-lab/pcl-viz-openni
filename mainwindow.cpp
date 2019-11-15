#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <time.h>
#include <random>
#include <QDebug>
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
ui(new Ui::MainWindow),
laserCloud(),
laserCloudOri(),
coeffSel()
{
    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/sukie/code/loam-feature-vis/cmake-build-debug/export/pcl-301.pcd", laserCloud) == -1
     || pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/sukie/code/loam-feature-vis/cmake-build-debug/export/ori-301.pcd", laserCloudOri) == -1
     || pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/sukie/code/loam-feature-vis/cmake-build-debug/export/eff-301.pcd", coeffSel) == -1)
    {
      PCL_ERROR ("Couldn't read PCD files\n");
      return;
    }

    ui->setupUi(this);
    glviewer = new GLWidget;
    ui->scrollArea->setWidget(glviewer);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(slot_timer()));
    connect(ui->pushButton_start, SIGNAL(clicked(bool)), this, SLOT(slot_btn_start()));
    connect(ui->pushButton_stop, SIGNAL(clicked(bool)), this, SLOT(slot_btn_stop()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::slot_timer()
{
    std::vector<DrawingElem> elems;
    DrawingElem oneelem;

    std::vector<point3fi> pos;
    std::vector<color3b> rgb;
    std::vector<int> siz;

    for (auto pt_: laserCloud){
        point3fi pt;
        color3b color;
        pt.x = pt_.x ;
        pt.y = pt_.y;
        pt.z = pt_.z;

        if (true) {
            color.r = color.g = color.b = 128;
            pos.push_back(pt);
            rgb.push_back(color);
            siz.push_back(2);
        }
    }
    oneelem.pts = pos;
    oneelem.colors = rgb;
    oneelem.pointsize = siz;
    oneelem.type = POINTS;
    elems.push_back(oneelem);


    pos.clear();
    rgb.clear();
    siz.clear();
    assert(laserCloudOri.points.size() == coeffSel.points.size());
    int featureNum = laserCloudOri.points.size();
    for (int idx = 1; idx < featureNum; idx++){
        point3fi pt;
        color3b color;
        pt.x = laserCloudOri.points[idx].x;
        pt.y = laserCloudOri.points[idx].y;
        pt.z = laserCloudOri.points[idx].z;
        genRandomColor(color);
        float siz_ = 12; // round(coeffSel.points[idx].z * coeffSel.points[idx].z * 50);
        if (true) {
            pos.push_back(pt);
            rgb.push_back(color);
            siz.push_back(siz_);
        }
    }
    oneelem.pts = pos;
    oneelem.colors = rgb;
    oneelem.pointsize = siz;
    oneelem.type = POINTS;
    elems.push_back(oneelem);

    glviewer->setPointCloudFrame(elems);
    slot_btn_stop();

}

void MainWindow::slot_btn_start()
{
    timer->start(50);
}

void MainWindow::slot_btn_stop()
{
    timer->stop();
}
