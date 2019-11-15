#include "velodynecalib.h"
#include <QFile>
#include <QDomDocument>
#include <fstream>
#include <qmath.h>
VeloInnerCalib::VeloInnerCalib() {
    linenum = 64;
}

int endsWith(std::string s, std::string sub){
        return s.rfind(sub)==(s.length()-sub.length())?1:0;
}

bool VeloInnerCalib::loadCalib(std::string filename) {
    return readXMLCalibFile(filename);
}

/**
 * @brief VeloInnerCalib::readXMLCalibFile for HDL-64 and HDL-32
 * @param filename
 * @return Velodyne标定文件中的距离单位cm，角度单位是degree
 */
bool VeloInnerCalib::readXMLCalibFile(std::string filename)
{
    for (size_t i = 0; i != linenum; ++i)
        idx.push_back(i);

    QFile file(filename.c_str());
    file.open(QFile::ReadOnly | QFile::Text);
    if (!file.isOpen()) {
        return false;
    }
    QDomDocument document;
    document.setContent(&file);
    QDomElement root = document.documentElement();
    // qDebug() << root.tagName();
    QDomNode DB = root.firstChild();
    QDomNodeList children = DB.childNodes();
    for (int i = 0; i < children.size(); i++) {
        QDomNode node = children.at(i);
        QString nodeName = node.nodeName();
        // qDebug() << nodeName;
        if (nodeName == "distLSB_") {
            QDomText text =  node.firstChild().toText();
            distLSB = text.data().toDouble();
            distLSB *=0.01;
        }
        if (nodeName == "position_") {
            QDomElement xyz = node.firstChildElement();
            xyz = xyz.firstChildElement("item");
            position_x = xyz.text().toDouble();
            xyz = xyz.nextSiblingElement("item");
            position_y = xyz.text().toDouble();
            xyz = xyz.nextSiblingElement("item");
            position_z = xyz.text().toDouble();
            //  qDebug() << position_x << position_y << position_z;
        }
        if (nodeName == "orientation_") {
            QDomElement rpy = node.firstChildElement();
            rpy = rpy.firstChildElement("item");
            orientation_r = rpy.text().toDouble();
            rpy = rpy.nextSiblingElement("item");
            orientation_p = rpy.text().toDouble();
            rpy = rpy.nextSiblingElement("item");
            orientation_y = rpy.text().toDouble();
        }
//        if (nodeName == "colors_") {
//            QDomElement item = node.firstChildElement("item");
//            while (!item.isNull()) {
//                QDomElement rgb = item.firstChildElement();
//                rgb = rgb.firstChildElement("item");
//                double r =  rgb.text().toDouble();
//                rgb = rgb.nextSiblingElement("item");
//                double g =  rgb.text().toDouble();
//                rgb = rgb.nextSiblingElement("item");
//                double b =  rgb.text().toDouble();
//                colors_r << r;
//                colors_g << g;
//                colors_b << b;
//                item = item.nextSiblingElement("item");
//            }
//        }
        if (nodeName == "enabled_") {
            QDomElement item = node.firstChildElement("item");
            while (!item.isNull()) {
                enabled.push_back(item.text() == "1");
                item = item.nextSiblingElement("item");
            }
        }
        if (nodeName == "intensity_") {
            QDomElement item = node.firstChildElement("item");
            while (!item.isNull()) {
                intensity.push_back(item.text() == "1");
                item = item.nextSiblingElement("item");
            }
        }
        if (nodeName == "minIntensity_") {
            QDomElement item = node.firstChildElement("item");
            while (!item.isNull()) {
                minIntensity.push_back(item.text().toDouble());
                item = item.nextSiblingElement("item");
            }
        }
        if (nodeName == "maxIntensity_") {
            QDomElement item = node.firstChildElement("item");
            while (!item.isNull()) {
                maxIntensity.push_back(item.text().toDouble());
                item = item.nextSiblingElement("item");
            }
        }
        if (nodeName == "points_") {
            QDomElement item = node.firstChildElement("item");
            double _rotCorrection, _vertCorrection,_distCorrection;
            double _distCorrectionX, _distCorrectionY, _vertOffsetCorrection;
            double _horizOffsetCorrection, _focalDistance, _focalSlope;
            while (!item.isNull()) {
                QDomElement px = item.firstChildElement();
                _rotCorrection = px.firstChildElement("rotCorrection_").text().toDouble();
                rotCorrection.push_back(qDegreesToRadians(_rotCorrection));

                _vertCorrection = px.firstChildElement("vertCorrection_").text().toDouble();
                vertCorrection.push_back( qDegreesToRadians(_vertCorrection) );

                _distCorrection = px.firstChildElement("distCorrection_").text().toDouble();
                distCorrection.push_back( _distCorrection/100.0 );

                _distCorrectionX = px.firstChildElement("distCorrectionX_").text().toDouble();
                distCorrectionX.push_back( _distCorrectionX/100.0 );

                _distCorrectionY = px.firstChildElement("distCorrectionY_").text().toDouble();
                distCorrectionY.push_back( _distCorrectionY/100.0 );

                _vertOffsetCorrection = px.firstChildElement("vertOffsetCorrection_").text().toDouble();
                vertOffsetCorrection.push_back( _vertOffsetCorrection/100.0 );

                _horizOffsetCorrection = px.firstChildElement("horizOffsetCorrection_").text().toDouble();
                horizOffsetCorrection.push_back( _horizOffsetCorrection/100.0 );

                _focalDistance = px.firstChildElement("focalDistance_").text().toDouble();
                focalDistance.push_back( _focalDistance/100.0 );

                focalSlope.push_back( px.firstChildElement("focalSlope_").text().toDouble());
                item = item.nextSiblingElement("item");
            }
        }
    }
    minAngle = vertCorrection[0];
    maxAngle = vertCorrection[0];
    for (int i = 1; i < vertCorrection.size(); i++) {
        if (vertCorrection[i] > maxAngle) {
            maxAngle = vertCorrection[i];
        }
        if (vertCorrection[i] < minAngle) {
            minAngle = vertCorrection[i];
        }
    }
//    for (int i = 0; i < vertCorrection.size(); i++) {
//        double r = (vertCorrection[i] - minAngle) / (maxAngle - minAngle);
//        double b = 1.0 - r;
//        double g = r > 0.5 ? b * 2 : r * 2;
//        colors_r << r;
//        colors_g << g;
//        colors_b << b;
//    }
//  qDebug()<<enabled.size(); //cdb maybe has a problem for vector template
//  qDebug()<<intensity.size();
    std::sort(idx.begin(), idx.end(), [this](size_t i1, size_t i2) {
        return vertCorrection[i1] <  vertCorrection[i2];
    });
    return true;
}
