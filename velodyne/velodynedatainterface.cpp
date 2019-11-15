#include "velodynedatainterface.h"
#include <cmath>
#include <string.h>
#include <ctime>
#include <iostream>
VelodyneDataInterface::VelodyneDataInterface()
{   
    frameSize = 1248;
}
VelodyneDataInterface::~VelodyneDataInterface()
{
    if (velodyne_file.is_open())
        velodyne_file.close();
}

VelodyneDataInterface::VelodyneDataInterface(std::string inner_calib_filename, std::string out_calib_filename)
{
    loadCalibParams(inner_calib_filename, out_calib_filename);
}

VeloInnerCalib &VelodyneDataInterface::getInnercalib()
{
    return innercalib;
}

void VelodyneDataInterface::loadCalibParams(std::string inner_calib_filename, std::string out_calib_filename)
{
    innercalib.loadCalib(inner_calib_filename);
}

void VelodyneDataInterface::loadInnerCalibParams(std::string inner_calib_filename)
{
    this->innercalib.loadCalib(inner_calib_filename);
}

void VelodyneDataInterface::loadOuterCalibParams(std::string out_calib_filename)
{

}

bool VelodyneDataInterface::loadData(std::string filename)
{
    if(filename == "")
        return 0;

    return pcaploader.loadPcapData(filename);
}

bool VelodyneDataInterface::getVeloRingData(std::vector<LaserPoint>& points)
{
    if (!pcaploader.isFileOpened())
        return 0;

    if (this->innercalib.enabled.size() == 0)
        return 0;

    points.clear();

    int oneloop_flag = 0;

    char *streams = new char[frameSize*2];
    char *data;

    double lastrotation;

    bool first_packet = 1;

    while (!pcaploader.getFileHeader().eof())
    {
        long long timestamp;

        if (oneloop_flag)
            break;

        PcapPktHeader header = {0};
        if (!pcaploader.getStreamData(streams, frameSize, header))
            continue;

        data = &streams[42]; // 1206 for one packet

        struct tm *p_tm;
        time_t timep(header.seconds);
        p_tm = localtime(&timep);
        timestamp = ((p_tm->tm_hour*60 + p_tm->tm_min)*60 + p_tm->tm_sec)*1000 + header.u_seconds/1000;

        for (int i=0; i<12; i++){
            int blockStart = i*100;
            int blockId = (unsigned char)data[blockStart] + (unsigned char)data[blockStart + 1] * 0x100;
            int rotationData =  (unsigned char)data[blockStart + 2] + (unsigned char)data[blockStart + 3] * 0x100;
            double theta = rotationData / 18000.0 * 3.1415926535;

            if (first_packet){
                first_packet = 0;
                lastrotation = rotationData;
            }

            for (int j=0; j<32; j++){
                int blockOffset = (blockId == 0xEEFF ? 0 : 32);
                int distanceData = (unsigned char)data[blockStart + 4 + j * 3] + (unsigned char)data[blockStart + 5 + j * 3] * 0x100;
                int intensityData = (unsigned char)data[blockStart + 6 + j * 3];

                double distance = distanceData * innercalib.distLSB + innercalib.distCorrection.at(j + blockOffset);

                double cosVertAngle = cos(innercalib.vertCorrection.at(j + blockOffset));
                double sinVertAngle = sin(innercalib.vertCorrection.at(j + blockOffset));
                double cosRotCorrection = cos(innercalib.rotCorrection.at(j + blockOffset));
                double sinRotCorrection = sin(innercalib.rotCorrection.at(j + blockOffset));

                double cosRotAngle = cos(theta) * cosRotCorrection + sin(theta) * sinRotCorrection;
                double sinRotAngle = sin(theta) * cosRotCorrection - cos(theta) * sinRotCorrection;

                double hOffsetCorr = innercalib.horizOffsetCorrection.at(j + blockOffset);
                double vOffsetCorr = innercalib.vertOffsetCorrection.at(j + blockOffset);

                double xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle;

                LaserPoint onepoint;
                onepoint.x = xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle;
                onepoint.y = xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle;
                onepoint.z = distance * sinVertAngle + vOffsetCorr * cosVertAngle;
                onepoint.distance = distance;
                onepoint.id = j + blockOffset;
                onepoint.rot = theta;
                onepoint.intensity = intensityData;
                onepoint.timestamp = timestamp;

                oneringpoints.push_back(onepoint);


                if (lastrotation > rotationData){
                    points = oneringpoints;
                    oneringpoints.clear();
                    oneloop_flag = 1;
                }

                lastrotation = rotationData;
            }

        }
        //parse gps
        //skip ----

    }
//    std::cout<<"-----------------------One Ring Data------------------------"<<std::endl;
    delete [] streams;
    return 1;
}


