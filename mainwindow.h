#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include "logparser.h"
#include "glwidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:
    void slot_timer();
    void slot_btn_start();
    void slot_btn_stop();
private:

    GLWidget *glviewer;
    LogParser logobj;
    QTimer *timer;

    pcl::PointCloud<pcl::PointXYZI> laserCloud;
    pcl::PointCloud<pcl::PointXYZI> laserCloudOri;
    pcl::PointCloud<pcl::PointXYZI> coeffSel;
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
