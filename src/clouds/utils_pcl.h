#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <LinearMath/btVector3.h>
#include "pcl_typedefs.h"

static const float cx = 320-.5;
static const float cy = 240-.5;
static const float f = 525;

typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
typedef Eigen::Matrix<uint8_t,Eigen::Dynamic,Eigen::Dynamic> MatrixXu;

ColorCloudPtr readPCD(const std::string& pcdfile);

Eigen::MatrixXi xyz2uv(const Eigen::MatrixXf& xyz);
inline cv::Point2f xyz2uv(const btVector3& point) {
  return cv::Point2f(f*point.x()/point.z() + cx, f*point.y()/point.z() + cy);
}

Eigen::MatrixXf toEigenMatrix(ColorCloudPtr);
Eigen::MatrixXf toEigenMatrix(const cv::Mat&);
Eigen::MatrixXf getDepthImage(ColorCloudPtr);
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(const std::vector< std::vector<float> >&);

MatrixXu toBGR(ColorCloudPtr);
cv::Mat toCVMat(Eigen::MatrixXf);
cv::Mat toCVMatImage(const ColorCloudPtr cloud);
cv::Mat toCVMatDepthImage(const ColorCloudPtr cloud);

bool pointIsFinite(const ColorPoint& pt);

inline ColorCloudPtr removeConst(ConstColorCloudPtr cloud) {
  return boost::const_pointer_cast< pcl::PointCloud<ColorPoint> >(cloud);
}
ColorCloudPtr transformPointCloud1(ColorCloudPtr in, Eigen::Affine3f transform);
ColorCloudPtr extractInds(ColorCloudPtr in, std::vector<int> inds);

bool saveTransform(const std::string& filename, const Eigen::Matrix4f& t);
bool loadTransform(const std::string& filename, Eigen::Matrix4f& t);
bool saveTransform(const std::string& filename, const Eigen::Affine3f& t);
bool loadTransform(const std::string& filename, Eigen::Affine3f& t);
