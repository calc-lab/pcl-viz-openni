#ifndef VELOCALIBMJL_H
#define VELOCALIBMJL_H
#include <vector>
#include <string>

/**
 * @brief 存储Velodyne的出场标定
 *
 */
class VeloInnerCalib {
public:
    VeloInnerCalib();
    bool loadCalib(std::string filename);

private:
    bool readXMLCalibFile(std::string filename);
    bool readTxtCalibFile(std::string filename);

public:
    int linenum; /**< 激光线数 */
    double distLSB; /**< 一单位表示的实际长度 */
    double position_x; /**< 激光x方向的平移量 */
    double position_y; /**< 激光y方向的平移量 */
    double position_z; /**< 激光z方向的平移量 */
    double orientation_r; /**< 激光roll方向的旋转 */
    double orientation_p; /**< 激光pitch方向的旋转 */
    double orientation_y; /**< 激光yaw方向的旋转 */
    // intrinsic paras
    std::vector<bool> enabled; /**< 激光某条线是否可用 */
    std::vector<bool> intensity; /**< 反射强度是否有效 */
    std::vector<double> minIntensity; /**< 最小反射强度 */
    std::vector<double> maxIntensity; /**< 最大反射强度 */
    std::vector<double> rotCorrection; /**< 旋转偏移量，参见Velodyne文档，左正右负 */
    std::vector<double> vertCorrection; /**< 俯仰偏移量，参见Velodyne文档，上正下负 */
    std::vector<double> distCorrection; /**< 距离偏移量，参见Velodyne文档 */
    std::vector<double> distCorrectionX; /**< 距离x方向偏移量，参见Velodyne文档 */
    std::vector<double> distCorrectionY; /**< 距离y方向偏移量，参见Velodyne文档 */
    std::vector<double> vertOffsetCorrection; /**< 纵向距离偏移量，参见Velodyne文档 */
    std::vector<double> horizOffsetCorrection; /**< 横向距离偏移量，参见Velodyne文档 */
    std::vector<double> focalDistance; /**< 焦距，参见Velodyne文档 */
    std::vector<double> focalSlope; /**< 焦点斜率，参见Velodyne文档 */
    double minAngle; /**< 最小俯仰角度 */
    double maxAngle; /**< 最大俯仰角度 */
    std::vector<size_t> idx; /**< 从上至下的激光线索引，0-63 */
};

#endif // VELOCALIBMJL_H
