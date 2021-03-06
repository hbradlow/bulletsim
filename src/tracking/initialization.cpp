#include <ros/topic.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "initialization.h"
#include "utils_tracking.h"
#include "config_tracking.h"
#include <geometry_msgs/Transform.h>
#include "simulation/config_bullet.h"
#include "utils/conversions.h"
#include <bulletsim_msgs/TrackedObject.h>
#include <bulletsim_msgs/Initialization.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include "tracked_object.h"
#include <tf/tf.h>
#include "simulation/bullet_io.h"
#include "simulation/softbodies.h"
#include "utils/logging.h"

//DEBUG
#include "physics_tracker.h"
#include "plotting_tracking.h"
#include "simulation/util.h"
using namespace Eigen;

using namespace std;

TrackedObject::Ptr toTrackedObject(const bulletsim_msgs::ObjectInit& initMsg, ColorCloudPtr cloud, ColorCloudPtr organized_cloud, cv::Mat image, CoordinateTransformer* transformer) {
  if (initMsg.type == "rope") {
	  vector<btVector3> nodes = toBulletVectors(initMsg.rope.nodes);
//		//downsample nodes
//		vector<btVector3> nodes;
//		for (int i=0; i<nodes_o.size(); i+=3)
//			nodes.push_back(nodes_o[i]);
	  BOOST_FOREACH(btVector3& node, nodes) node += btVector3(0,0,.01);

	  CapsuleRope::Ptr sim(new CapsuleRope(scaleVecs(nodes,METERS), initMsg.rope.radius*METERS));
	  TrackedRope::Ptr tracked_rope(new TrackedRope(sim));
		cv::Mat tex_image = tracked_rope->makeTexture(cloud);
		sim->setTexture(tex_image);

	  return tracked_rope;
  }
//  else if (initMsg.type == "towel_corners") {
//	  vector<btVector3> poly_corners = polyCorners(organized_cloud);
//	  BulletSoftObject::Ptr sim = makeSponge(poly_corners, 0.05*METERS, 4.0*METERS*METERS*METERS/1000000.0, 300);
//	  sim->softBody->translate(btVector3(0,0,0.1*METERS));
//
//	  TrackedCloth::Ptr tracked_towel(new TrackedCloth(sim, cv::Mat(), 10, 10, 10, 10));
//	  return tracked_towel;
//  }
  else if (initMsg.type == "towel_corners") {
	  const vector<geometry_msgs::Point32>& points = initMsg.towel_corners.polygon.points;
	  vector<btVector3> corners = scaleVecs(toBulletVectors(points),METERS);

	  float sx = (corners[0] - corners[1]).length();
		float sy = (corners[0] - corners[3]).length();
		int resolution_x = sx/(TrackingConfig::node_distance*METERS) + 1;
		int resolution_y = sy/(TrackingConfig::node_distance*METERS) + 1;
		float mass = (TrackingConfig::surface_density/(METERS*METERS)) * (sx * sy);

	  printf("Created towel with following properties:\n");
	  printf("Surface density (Mass per area): %f\n", TrackingConfig::surface_density);
	  printf("Mass: %f\n", mass);
	  printf("Dimensions and area: %f x %f = %f\n", sx/METERS, sy/METERS, sx*sy/(METERS*METERS));
	  printf("Node distance (distance between nodes): %f\n", TrackingConfig::node_distance);
	  printf("Resolution: %d %d\n", resolution_x, resolution_y);

	  vector<btVector3> poly_corners = polyCorners(cloud, image, transformer);
	  //BOOST_FOREACH(btVector3& poly_corner, poly_corners) util::drawSpheres(poly_corner, Vector3f(1,0,0), 0.5, 2, env);
  	BulletSoftObject::Ptr sim = makeCloth(poly_corners, resolution_x, resolution_y, mass);
	  sim->setTexture(image, toBulletTransform(transformer->camFromWorldEigen));

	  //Shift the whole cloth upwards in case some of it starts below the table surface
	  sim->softBody->translate(btVector3(0,0,0.01*METERS));

	  //for (int i=0; i<sim->softBody->m_nodes.size(); i++) {
//		for (int i=0; i<10; i++) {
//			util::drawSpheres(sim->softBody->m_nodes[i].m_x, Vector3f(1,0,0), 0.5, 2, env);
//	  	cv::Point2f pixel = sim->getTexCoord(i);
//	  	image.at<cv::Vec3b>(pixel.y, pixel.x) = cv::Vec3b(255,255,255);
//	  }
//	  cv::imwrite("/home/alex/Desktop/tshirt_tex2.jpg", image);

	  TrackedCloth::Ptr tracked_towel(new TrackedCloth(sim, image, resolution_x, resolution_y, sx, sy));

	  return tracked_towel;
  }
  else if (initMsg.type == "box") {
	  btScalar mass = 1;
	  btVector3 halfExtents = toBulletVector(initMsg.box.extents)*0.5*METERS;
	  Eigen::Matrix3f rotation = (Eigen::Matrix3f) Eigen::AngleAxisf(initMsg.box.angle, Eigen::Vector3f::UnitZ());
	  btTransform initTrans(toBulletMatrix(rotation), (toBulletVector(initMsg.box.center) + btVector3(0,0,.15))*METERS);
	  BoxObject::Ptr sim(new BoxObject(mass, halfExtents, initTrans));
	  TrackedBox::Ptr tracked_box(new TrackedBox(sim));
		sim->setColor(1,0,0,1);

	  return tracked_box;
  }
  else
	  throw runtime_error("unrecognized initialization type" + initMsg.type);
}

bulletsim_msgs::TrackedObject toTrackedObjectMessage(TrackedObject::Ptr obj) {
  bulletsim_msgs::TrackedObject msg;
  if (obj->m_type == "rope") {
    msg.type = obj->m_type;
    msg.rope.nodes = toROSPoints(scaleVecs(obj->getPoints(), 1/METERS));
  }
  else {
	  //TODO
	  //LOG_ERROR("I don't knot how to publish a ");
  }
  return msg;
}

TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr cloud, ColorCloudPtr organized_cloud, cv::Mat image, CoordinateTransformer* transformer) {
  bulletsim_msgs::Initialization init;
  pcl::toROSMsg(*scaleCloud(cloud, 1/METERS), init.request.cloud);
  init.request.cloud.header.frame_id = "/ground";
	
  bool success = ros::service::call(initializationService, init);
  if (success)
  	return toTrackedObject(init.response.objectInit, cloud, organized_cloud, image, transformer);
  else {
		ROS_ERROR("initialization failed");
		return TrackedObject::Ptr();
  }
}

TrackedObject::Ptr callInitServiceAndCreateObject(ColorCloudPtr cloud, cv::Mat image, CoordinateTransformer* transformer) {
	return callInitServiceAndCreateObject(scaleCloud(cloud, METERS), ColorCloudPtr(new ColorCloud()), image, transformer);
}
