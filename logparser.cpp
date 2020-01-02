#include "logparser.h"
#include <QDir>
#include <QMessageBox>
#include <QFile>
#include <QtMath>
#include <QStringList>
#include <QDebug>

static inline double dist(double x0,double y0,double z0,double x1,double y1,double z1){
    return qSqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1)+(z0-z1)*(z0-z1));
}
LogParser::LogParser() {
    _laserCloudFullRes.clear();
    _lineFeaturePts.clear();
    _planeFeaturePts.clear();
}

bool LogParser::loadLog(std::string path) {
    QString fLaserCloudFullRes = QString(path.c_str()) + "/laserCloudFullRes.pcd";
    QString fLineFeaturePts = QString(path.c_str()) + "/cornerPointsSharp.pcd";
    QString fPlaneFeaturePts = QString(path.c_str()) + "/surfPointsFlat.pcd";
    pcl::PointCloud<pcl::PointXYZI> pLaserCloudFullRes;
    pcl::PointCloud<pcl::PointXYZI> pLineFeaturePts;
    pcl::PointCloud<pcl::PointXYZI> pPlaneFeaturePts;
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (fLaserCloudFullRes.toStdString(), pLaserCloudFullRes) == -1
     || pcl::io::loadPCDFile<pcl::PointXYZI> (fLineFeaturePts.toStdString(), pLineFeaturePts) == -1
     || pcl::io::loadPCDFile<pcl::PointXYZI> (fPlaneFeaturePts.toStdString(), pPlaneFeaturePts) == -1) {
        QMessageBox::warning(nullptr, "ERROR", "Invalid input files");
        return false;
    }
    for (auto p: pLaserCloudFullRes) {
        point4d q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.info = p.intensity;
        _laserCloudFullRes.push_back(q);
    }
    for (auto p: pLineFeaturePts) {
        point4d q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.info = p.intensity;
        _lineFeaturePts.push_back(q);
    }
    for (auto p: pPlaneFeaturePts) {
        point4d q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.info = p.intensity;
        _planeFeaturePts.push_back(q);
    }


    return  true;
}

std::vector<point4d> LogParser::laserCloudFullRes() const {
    return _laserCloudFullRes;
}

std::vector<point4d> LogParser::lineFeaturePts() const {
    return _lineFeaturePts;
}

std::vector<point4d> LogParser::planeFeaturePts() const {
    return _planeFeaturePts;
}
