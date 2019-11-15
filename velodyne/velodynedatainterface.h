#ifndef VELODYNEDATAINTERFACE_H
#define VELODYNEDATAINTERFACE_H
#include <fstream>
#include "pcapparser.h"
#include "define.h"
#include "velodynecalib.h"
struct LaserPoint
{
    double x,y,z;
    double rot;
    short id;
    long long timestamp; //ms
    double distance;
    short intensity;
};
struct VelodynePacket
{
    long long timestamp;
    std::vector<double> distances;
    std::vector<double> intensity;
    std::vector<point3d> pts;
    void clear()
    {
        timestamp = 0;
        distances.clear();
        intensity.clear();
        pts.clear();
    }
    void operator =(const VelodynePacket & packet)
    {
        timestamp = packet.timestamp;
        distances = packet.distances;
        intensity = packet.intensity;
        pts       = packet.pts;
    }
};

class VelodyneDataInterface
{
public:
    VelodyneDataInterface();
    ~VelodyneDataInterface();
    VelodyneDataInterface(std::string inner_calib_filename, std::string out_calib_filename);

public:
    void loadCalibParams(std::string inner_calib_filename, std::string out_calib_filename);
    void loadInnerCalibParams(std::string inner_calib_filename);
    void loadOuterCalibParams(std::string out_calib_filename);
    bool loadData(std::string filename);

    bool getVeloRingData(std::vector<LaserPoint>& points);

    bool getFrame(long long &timestamp, long long &timestamp_utc, std::vector<rgbpoint3d> &points,  std::vector<double>& dist, bool output_dist);
    void setRange(double start_angle, double stop_angle);
    void getRange(double& start_angle, double& stop_angle);

    VeloInnerCalib &getInnercalib();

private:
    VeloInnerCalib innercalib;

    PcapParser pcaploader;

    std::ifstream velodyne_file;

    std::vector<LaserPoint> oneringpoints;

    double startAngle;
    double stopAngle;

    int frameSize;
};

#endif // VELODYNEDATAINTERFACE_H
