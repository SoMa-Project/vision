/**
 * @author Can Erdogan
 * @date 2017-05-23
 * @brief Reads the table_fits/polygons marker array message from vision code and 
 * creates a WRL file for CERRT.
 * NOTE:
 * 1- Removes any plane that is not connected to any of the others.
 * 2- Assumes a fixed camera location to convert the points into world coordinates.
 * 3- Assumes that polygon vertices are ordered so that the normals are look towards camera.
 */

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <Eigen/Dense>
#include <set>

#define PLANE_DIST 0.05
#define EDGE_DIST 0.05

using namespace std;
using namespace Eigen;

/* ******************************************************************************************** */
void chatterCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {

	// Create the camera transform in the world frame
	Quaternion <double> camQ (-0.340, 0.790, -0.469, 0.202);
	Matrix3d camR = camQ.toRotationMatrix();
  Vector3d camT (0.000, -0.700, 1.400);
	cout << camR << endl;

	// Get the polygon data
	vector < vector <Vector3d> > polygons;
	vector <Vector3d> means;
	for(int i = 0; i < msg->markers.size(); i++) {

		// Check if this marker contains polygon data
		const visualization_msgs::Marker& m = msg->markers[i];
		if(m.ns.compare("table_fits/polygon") == 1) {

			// Save the list of points 
			vector <Vector3d> poly;
			cout  << "0 0 0" << endl;
			Vector3d mean (0,0,0);
			for(int j = 0; j < m.points.size(); j++) {
				Vector3d loc = Vector3d(m.points[j].x, m.points[j].y, m.points[j].z);
				Vector3d worldP = camR * loc + camT;
				cout <<  worldP.transpose() << endl;
				poly.push_back(worldP);
				if(j < (m.points.size() - 1)) mean += worldP;
			}
			mean /= (m.points.size() - 1);
			means.push_back(mean);
			polygons.push_back(poly);
		}
	}

	// Compute the plane parameters for each polygon
	cout  << "0 0 0" << endl;
	vector <Vector4d> planes;
	for(int poly = 0; poly < polygons.size(); poly++) {
		Vector3d p1 = polygons[poly][0];
		Vector3d p2 = polygons[poly][1];
		Vector3d p3 = polygons[poly][2];
		Vector3d d21 = (p2-p1).normalized();
		Vector3d d31 = (p3-p1).normalized();
		Vector3d n = (d21.cross(d31)).normalized();
		double d = -n.dot(p1);
		planes.push_back(Vector4d(n(0), n(1), n(2), d));
		cout << n.transpose() << endl;
	}

	// For each polygon, create a shape in wrl file
	set <int> connections;
	for(int poly1 = 0; poly1 < polygons.size(); poly1++) {
		printf("\t\tTransform {\n
      translation 
      children 
				Shape {
					appearance Appearance{
						material Material { 
							diffuseColor     1 0 0 #simple red
							}
						}
					geometry IndexedFaceSet {
						coord Coordinate {
														point [
	
	}

  ROS_INFO("I heard! %d\n", polygons.size());
	exit(1);
}

/* ******************************************************************************************** */
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/plasm_yaml_ros_node/ecto_markers", 1000, chatterCallback);
  ros::spin();
  return 0;
}
/* ******************************************************************************************** */
