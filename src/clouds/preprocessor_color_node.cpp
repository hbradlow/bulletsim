#include <cmath>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "clouds/utils_pcl.h"
#include "clouds/cloud_ops.h"
#include "get_table2.h"
#include "utils_cv.h"
#include "utils/my_exceptions.h"
#include "utils/config.h"
#include "utils/conversions.h"
#include "utils_ros.h"
#include "utils/file.h"

using namespace std;
using namespace Eigen;

static std::string nodeNS = "/preprocessor";
static std::string nodeName = "/preprocessor_node";

enum boundary_t { NONE, LOAD_FILE, RED_MARKERS, TABLE };

struct LocalConfig : Config {
  static std::string nodeNS;
  static std::string inputTopic;
  static std::string nodeName;
  static float zClipLow;
  static float zClipHigh;
  static bool updateParams;
  static float downsample;
  static bool removeOutliers;
  static float clusterTolerance;
  static float clusterMinSize;
  static float outlierRadius;
  static int outlierMinK;
  static float offset;
  static string boundaryFile;
  static int boundaryType;
  static bool debugMask;
  static bool debugGreenFilter;
  static int i0;
  static int i1;
  static int i2;
  static int i3;

  LocalConfig() : Config() {
    params.push_back(new Parameter<string>("inputTopic", &inputTopic, "input topic"));
    params.push_back(new Parameter<float>("zClipLow", &zClipLow, "clip points that are less than this much above table"));
    params.push_back(new Parameter<float>("zClipHigh", &zClipHigh, "clip points that are more than this much above table"));
    params.push_back(new Parameter<bool>("updateParams", &updateParams, "start a thread to periodically update the parameters thru the parameter server"));
    params.push_back(new Parameter<float>("downsample", &downsample, "downsample voxel grid size. 0 means no"));
    params.push_back(new Parameter<bool>("removeOutliers", &removeOutliers, "remove outliers"));
    params.push_back(new Parameter<float>("clusterTolerance", &clusterTolerance, "points within this distance are in the same cluster"));
    params.push_back(new Parameter<float>("clusterMinSize", &clusterMinSize, "the clusters found must have at least this number of points. 0 means no filtering"));
    params.push_back(new Parameter<float>("outlierRadius", &outlierRadius, "radius search RadiusOutlierRemoval filter"));
    params.push_back(new Parameter<int>("outlierMinK", &outlierMinK, "minimum neighbors in radius search for RadiusOutlierRemoval filter"));
    params.push_back(new Parameter<float>("offset", &offset, "offset for the box filter (shrinks the box by offset at each side of the x and y axes)"));
    params.push_back(new Parameter<string>("boundaryFile", &boundaryFile, "file from which the boundary gets loaded/saved"));
    params.push_back(new Parameter<int>("boundaryType", &boundaryType, "the vertices defining the prism convex hull filter could be: NONE=0, LOAD_FILE, RED_MARKERS, TABLE"));
    params.push_back(new Parameter<bool>("debugMask", &debugMask, "set to true if you want to debug the intermediate mask filters"));
    params.push_back(new Parameter<bool>("debugGreenFilter", &debugGreenFilter, "set to true if you want to debug the lab threshold parameters. if true, the green and negative green cloud are published, and you can interactively modify the thresholds."));
    params.push_back(new Parameter<int>("i0", &i0, "miscellaneous variable 0"));
    params.push_back(new Parameter<int>("i1", &i1, "miscellaneous variable 1"));
    params.push_back(new Parameter<int>("i2", &i2, "miscellaneous variable 2"));
    params.push_back(new Parameter<int>("i3", &i3, "miscellaneous variable 3"));
  }
};

string LocalConfig::inputTopic = "/kinect1/depth_registered/points";
float LocalConfig::zClipLow = -0.02;
float LocalConfig::zClipHigh = 0.5;
bool LocalConfig::updateParams = true;
float LocalConfig::downsample = .02;
bool LocalConfig::removeOutliers = false;
float LocalConfig::clusterTolerance = 0.03;
float LocalConfig::clusterMinSize = 0;
float LocalConfig::outlierRadius = 0.02;
int LocalConfig::outlierMinK = 0;
float LocalConfig::offset = 0.02;
string LocalConfig::boundaryFile = "polygon";
int LocalConfig::boundaryType = LOAD_FILE;
bool LocalConfig::debugMask = false;
bool LocalConfig::debugGreenFilter = false;
int LocalConfig::i0 = 0;
int LocalConfig::i1 = 0;
int LocalConfig::i2 = 0;
int LocalConfig::i3 = 0;

static int MIN_L=0, MAX_L=255, MIN_A=115, MAX_A=255, MIN_B=0, MAX_B=255;

class PreprocessorNode {
public:
  ros::NodeHandle& m_nh;
  ros::Publisher m_cloudPub, m_imagePub;
  ros::Publisher m_cloudGreenPub, m_cloudNotGreenPub;
  ros::Publisher m_cloudHullPub;
  ros::Publisher m_polyPub;
  tf::TransformBroadcaster m_broadcaster;
  tf::TransformListener m_listener;
  ros::Subscriber m_sub;

  bool m_inited;
  bool m_frame_exists;

  Matrix3f m_axes;
  Vector3f m_mins, m_maxes;
	
  btTransform m_transform;
  geometry_msgs::Polygon m_poly;

  //DEBUG
  ColorCloudPtr cloud_hull;
  std::vector<pcl::Vertices> hull_vertices;

  void callback(const sensor_msgs::PointCloud2& msg_in) {
  	// Needs this to update the opencv windows
    if (LocalConfig::debugMask || LocalConfig::debugGreenFilter) {
    	char key = cv::waitKey(20);
    	if (key == 'q')
    		exit(0);
    }

  	ColorCloudPtr cloud_in(new ColorCloud());
    pcl::fromROSMsg(msg_in, *cloud_in);

    if (!m_inited) {
			if (LocalConfig::boundaryType != NONE) initPrismBoundaries(cloud_in, msg_in);
			m_inited = true;
		}

    // prism filter
    ColorCloudPtr cloud_out (new ColorCloud);
    if (!cloud_hull->empty()) cloud_out = cropToHull(cloud_in, cloud_hull, hull_vertices, true);
    else *cloud_out = *cloud_in;

    // image-based filters: color, morphological and connected components
		cv::Mat image = toCVMatImage(cloud_in);
		if (LocalConfig::debugMask) cv::imshow("cloud image", image);
		cv::Mat neg_green = colorSpaceMask(image, MIN_L, MAX_L, MIN_A, MAX_A, MIN_B, MAX_B, CV_BGR2Lab); // remove green
		//cv::Mat yellow = colorSpaceMask(image, 0, 255, 0, 255, 0, 123, CV_BGR2Lab); // add yellow
		cv::Mat yellow = colorSpaceMask(image, MIN_L, MAX_L, MIN_A, MAX_A, MIN_B, MAX_B, CV_BGR2Lab); // add yellow
		if (LocalConfig::debugMask) cv::imshow("mask negative green", neg_green);
		if (LocalConfig::debugMask) cv::imshow("mask yellow", yellow);

		cv::Mat cropped_image = toCVMatImage(cloud_out);
		if (LocalConfig::debugMask) cv::imshow("cropped image", cropped_image);

		cv::Mat mask = (neg_green | yellow) & toBinaryMask(cropped_image);
		if (LocalConfig::debugMask) cv::imshow("cropped image binary", cropped_image);

		if (LocalConfig::debugMask) cv::imshow("mask original", mask);
		mask = sparseSmallFilter(mask, 1, 2, 100, 2);
		if (LocalConfig::debugMask) cv::imshow("mask", mask);

		cv::Mat inv_mask = mask == 0;
		if (LocalConfig::debugMask) cv::imshow("inv_mask original", inv_mask);
		inv_mask = sparseSmallFilter(inv_mask, 1, 2, 100, 2);
		if (LocalConfig::debugMask) cv::imshow("inv_mask", inv_mask);

		mask = inv_mask == 0;
		// For some reason, there is white borders on this mask image; remove them
		mask.col(0) *= 0;
		mask.col(mask.cols-1) *= 0;
		mask.row(0) *= 0;
		mask.row(mask.rows-1) *= 0;
		if (LocalConfig::debugMask) cv::imshow("mask final", mask);

		cloud_out = maskCloud(cloud_in, mask);

		cv::merge(vector<cv::Mat>(3, mask), mask);
		image &= mask;

		if (LocalConfig::removeOutliers) cloud_out = removeOutliers(cloud_out, 1, 10);
		if (LocalConfig::downsample > 0) cloud_out = downsampleCloud(cloud_out, LocalConfig::downsample);
		if (LocalConfig::outlierMinK > 0) cloud_out = removeRadiusOutliers(cloud_out, LocalConfig::outlierRadius, LocalConfig::outlierMinK);
		if (LocalConfig::clusterMinSize > 0) cloud_out = clusterFilter(cloud_out, LocalConfig::clusterTolerance, LocalConfig::clusterMinSize);

		//Publish cloud
    sensor_msgs::PointCloud2 msg_out;
		pcl::toROSMsg(*cloud_out, msg_out);
		msg_out.header = msg_in.header;
		m_cloudPub.publish(msg_out);

		if (LocalConfig::debugGreenFilter) {
			cv::namedWindow("lab params");
			cv::createTrackbar("min l (lightness)     ", "lab params", &MIN_L, 255);
			cv::createTrackbar("max l (lightness)     ", "lab params", &MAX_L, 255);
			cv::createTrackbar("min a (green -> red)  ", "lab params", &MIN_A, 255);
			cv::createTrackbar("max a (green -> red)  ", "lab params", &MAX_A, 255);
			cv::createTrackbar("min b (blue -> yellow)", "lab params", &MIN_B, 255);
			cv::createTrackbar("max b (blue -> yellow)", "lab params", &MAX_B, 255);

			//Publish green cloud
			ColorCloudPtr cloud_green = colorSpaceFilter(cloud_in, MIN_L, MAX_L, MIN_A, MAX_A, MIN_B, MAX_B, CV_BGR2Lab, true, true);
			sensor_msgs::PointCloud2 msg_green;
			pcl::toROSMsg(*cloud_green, msg_green);
			msg_green.header = msg_in.header;
			m_cloudGreenPub.publish(msg_green);

			//Publish green cloud
			ColorCloudPtr cloud_not_green = colorSpaceFilter(cloud_in, MIN_L, MAX_L, MIN_A, MAX_A, MIN_B, MAX_B, CV_BGR2Lab, true, false);
			sensor_msgs::PointCloud2 msg_not_green;
			pcl::toROSMsg(*cloud_not_green, msg_not_green);
			msg_not_green.header = msg_in.header;
			m_cloudNotGreenPub.publish(msg_not_green);
		}

		//Publish image version of cloud
    cv_bridge::CvImage image_msg;
    image_msg.header   = msg_in.header;
    image_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    image_msg.image    = image;
    m_imagePub.publish(image_msg.toImageMsg());

		//broadcastKinectTransform(m_transform.inverse(), msg_in.header.frame_id, "ground", m_broadcaster, m_listener);

    geometry_msgs::PolygonStamped polyStamped;
    polyStamped.polygon = m_poly;
    polyStamped.header.frame_id = "/ground";
    polyStamped.header.stamp = ros::Time::now();
    m_polyPub.publish(polyStamped);
  }

  void initPrismBoundaries(ColorCloudPtr cloud, const sensor_msgs::PointCloud2& msg_in) {
  	if (LocalConfig::boundaryType == LOAD_FILE) {
    	loadPoints(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/boundary/" + LocalConfig::boundaryFile, m_poly.points);
    	// transform the poly points from ground to camera frame
    	btTransform transform = waitForAndGetTransform(m_listener, "/ground", msg_in.header.frame_id).inverse();
    	cloud_hull->resize(m_poly.points.size());
    	for (int i=0; i<m_poly.points.size(); i++)
    		cloud_hull->at(i) = toColorPoint( transform * toBulletVector(m_poly.points[i]) );
    	hull_vertices.resize(1);
    	for (int i=0; i<cloud_hull->size(); i++)
    		hull_vertices[0].vertices.push_back(i);
    	hull_vertices[0].vertices.push_back(0);

  	} else {

  		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			ColorCloudPtr cloud_filtered = filterPlane(cloud, 0.01, coefficients);
			ColorCloudPtr cloud_projected = projectOntoPlane(cloud_filtered, coefficients); // full table cloud

			if (LocalConfig::boundaryType == RED_MARKERS)
				cloud_projected = colorSpaceFilter(cloud_projected, 0,255,0,166,0,255, CV_BGR2Lab, false, true); // red table cloud

			cloud_hull = findConvexHull(cloud_projected, hull_vertices);

			// transform the poly points from camera to ground frame
			btTransform transform = waitForAndGetTransform(m_listener, "/ground", msg_in.header.frame_id);
			m_poly.points.resize(cloud_hull->size());
			for (int i=0; i<cloud_hull->size(); i++)
				m_poly.points[i] = toROSPoint32( transform * toBulletVector(cloud_hull->at(i)) );

	  	savePoints(string(getenv("BULLETSIM_SOURCE_DIR")) + "/data/boundary/" + LocalConfig::boundaryFile, m_poly.points);
  	}
  }

  void initTable(ColorCloudPtr cloud) {
    MatrixXf corners = getTableCornersRansac(cloud);

    Vector3f xax = corners.row(1) - corners.row(0);
    xax.normalize();
    Vector3f yax = corners.row(3) - corners.row(0);
    yax.normalize();
    Vector3f zax = xax.cross(yax);

    //if chess_board frame id exists, then z axis is already pointing up
    float zsgn = (m_listener.frameExists("chess_board") || (zax(2) < 0)) ? 1 : -1;
    xax *= zsgn;
    zax *= zsgn; // so z axis points up

    m_axes.col(0) = xax;
    m_axes.col(1) = yax;
    m_axes.col(2) = zax;

    MatrixXf rotCorners = corners * m_axes;

    m_mins = rotCorners.colwise().minCoeff();
    m_maxes = rotCorners.colwise().maxCoeff();
    m_mins(0) += LocalConfig::offset;
    m_mins(1) += LocalConfig::offset;
    m_maxes(0) -= LocalConfig::offset;
    m_maxes(1) -= LocalConfig::offset;
    m_mins(2) = rotCorners(0,2) + LocalConfig::zClipLow;
    m_maxes(2) = rotCorners(0,2) + LocalConfig::zClipHigh;

    m_transform.setBasis(toBulletMatrix(m_axes));
    m_transform.setOrigin(btVector3(corners(0,0), corners(0,1), corners(0,2)));

    m_poly.points = toROSPoints32(toBulletVectors(corners));

    m_inited = true;
  }

  PreprocessorNode(ros::NodeHandle& nh) :
    m_inited(false),
    m_nh(nh),
    m_cloudPub(nh.advertise<sensor_msgs::PointCloud2>(nodeNS+"/points",5)),
    m_imagePub(nh.advertise<sensor_msgs::Image>(nodeNS+"/image",5)),
    m_polyPub(nh.advertise<geometry_msgs::PolygonStamped>(nodeNS+"/polygon",5)),
    m_sub(nh.subscribe(LocalConfig::inputTopic, 1, &PreprocessorNode::callback, this)),
		m_mins(-10,-10,-10),
		m_maxes(10,10,10),
		m_transform(toBulletTransform(Affine3f::Identity())),
		cloud_hull(new ColorCloud)
    {
			if (LocalConfig::debugGreenFilter) {
				m_cloudGreenPub = nh.advertise<sensor_msgs::PointCloud2>(nodeNS+"/green/points",5);
				m_cloudNotGreenPub = nh.advertise<sensor_msgs::PointCloud2>(nodeNS+"/not_green/points",5);
			}
			m_cloudHullPub = nh.advertise<sensor_msgs::PointCloud2>(nodeNS+"/hull/points",5);
    }
};

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  ros::init(argc, argv,"preprocessor");
  ros::NodeHandle nh(nodeNS);

  PreprocessorNode tp(nh);
  ros::spin();
}
