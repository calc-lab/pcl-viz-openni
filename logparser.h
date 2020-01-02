#ifndef LOGPARSER_H
#define LOGPARSER_H

#include <vector>
#include <fstream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "define.h"

class LogParser
{
public:
    LogParser();
    bool loadLog(std::string _root);

    std::vector<point4d>  laserCloudFullRes() const;
    std::vector<point4d> lineFeaturePts() const;
    std::vector<point4d> planeFeaturePts() const;

    void setLineFeaturePts(const std::vector<point4d> &lineFeaturePts);

private:
    std::vector <point4d> _laserCloudFullRes;
    std::vector<point4d> _lineFeaturePts;
    std::vector<point4d> _planeFeaturePts;

};

#endif // LOGPARSER_H
