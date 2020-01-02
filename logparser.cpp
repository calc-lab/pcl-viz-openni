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
LogParser::LogParser()
{
    rawptbuf.clear();
    linefeatbuf.clear();
    planefeatbuf.clear();
}

bool LogParser::loadLog(std::string rawptfilename, std::string featfilename)
{
    QDir dir_test;
    if (!dir_test.exists(QString(rawptfilename.c_str())) || !dir_test.exists(featfilename.c_str())){
        QMessageBox::warning(nullptr, "ERROR", "Invalid log file");
        return false;
    }

    QFile rawptfile(rawptfilename.c_str());
    if (!rawptfile.open(QIODevice::ReadOnly | QIODevice::Text))
        return  false;

    while(!rawptfile.atEnd()){
        QString line(rawptfile.readLine());
        if (line.size()==0)
            break;
        QStringList items = line.split(",", QString::SkipEmptyParts);
        if (items.size()==4){
            point4d onept;
            onept.x = items.at(0).toDouble();
            onept.y = items.at(1).toDouble();
            onept.z = items.at(2).toDouble();
            onept.info = items.at(3).toDouble();
            rawptbuf.push_back(onept);
        }
    }

    QFile featfile(featfilename.c_str());
    if (!featfile.open(QIODevice::ReadOnly | QIODevice::Text))
        return  false;

    while(!featfile.atEnd()){
        QString line(featfile.readLine());
        if (line.size()==0)
            break;
        QStringList items = line.split(",", QString::SkipEmptyParts);
        if (items.size()==15){
            linefeat oneline;
            oneline.pt.x = items[1].toDouble();
            oneline.pt.y = items[2].toDouble();
            oneline.pt.z = items[3].toDouble();
            oneline.pt.info = items[4].toDouble();
            oneline.linehead.x = items[6].toDouble();
            oneline.linehead.y = items[7].toDouble();
            oneline.linehead.z = items[8].toDouble();
            oneline.linehead.info = items[9].toDouble();
            oneline.linetail.x = items[11].toDouble();
            oneline.linetail.y = items[12].toDouble();
            oneline.linetail.z = items[13].toDouble();
            oneline.linetail.info = items[14].toDouble();
            qDebug()<<dist(oneline.linehead.x, oneline.linehead.y,oneline.linehead.z,oneline.linetail.x,oneline.linetail.y,oneline.linetail.z);
            linefeatbuf.push_back(oneline);
        }
        else if (items.size()==20){
            planefeat oneplane;
            oneplane.pt.x = items[1].toDouble();
            oneplane.pt.y = items[2].toDouble();
            oneplane.pt.z = items[3].toDouble();
            oneplane.pt.info = items[4].toDouble();
            oneplane.plane_pt1.x = items[6].toDouble();
            oneplane.plane_pt1.y = items[7].toDouble();
            oneplane.plane_pt1.z = items[8].toDouble();
            oneplane.plane_pt1.info = items[9].toDouble();
            oneplane.plane_pt2.x = items[11].toDouble();
            oneplane.plane_pt2.y = items[12].toDouble();
            oneplane.plane_pt2.z = items[13].toDouble();
            oneplane.plane_pt2.info = items[14].toDouble();
            oneplane.plane_pt3.x = items[16].toDouble();
            oneplane.plane_pt3.y = items[17].toDouble();
            oneplane.plane_pt3.z = items[18].toDouble();
            oneplane.plane_pt3.info = items[19].toDouble();
            planefeatbuf.push_back(oneplane);
        }
    }
    return  true;
}

std::vector<point4d> LogParser::getRawptbuf() const
{
    return rawptbuf;
}

std::vector<linefeat> LogParser::getLinefeatbuf() const
{
    return linefeatbuf;
}

std::vector<planefeat> LogParser::getPlanefeatbuf() const
{
    return planefeatbuf;
}
