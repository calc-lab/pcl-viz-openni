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
    void draw();
    void slotPlayPause();
    void slotExit();
    void slotNextFrame();
    void slotPreviousFrame();
    void slotSkipForward();
    void slotSkipBackward();
private:
    void loadPointCloud();
    void drawSpecifiedFrame();

private:
    Ui::MainWindow *ui;
    GLWidget *glviewer;
    LogParser logobj;
    QTimer *timer;

    QString _root;
    int _lowerBound;
    int _upperBound;
    int _frameCounter;

    pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZI> _surfPointsFlat;
    pcl::PointCloud<pcl::PointXYZI> _laserCloudFullRes;
};

#endif // MAINWINDOW_H
