/**
 * @author Can Erdogan
 * @date 2017-05-19
 * @brief Reads the table_fits/polygons marker array message from vision code and 
 * outputs plane descriptions for CERRT.
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
/// Checks whether the projection of a given point onto the given plane is within the given
/// polygon
bool checkWithinPoly(const Vector3d& p1, const Vector4d& plane2, const vector <Vector3d>& poly2) {

	// First project the point
	Vector3d planeNormal = plane2.block<3,1>(0,0);
	double dist1 = p1.dot(planeNormal) + plane2(3);
	Vector3d p1p = p1 - dist1 * planeNormal;

	// For each edge, compute the outward normal
	double maxDist = -100;
	for(int vert2 = 0; vert2 < poly2.size(); vert2++) {

		// Get the vector from one vertex to another
		Vector3d q1 = poly2[vert2];
		Vector3d q2 = poly2[vert2 + 1];
		Vector3d d21 = (q2 - q1).normalized();

		// Get edge normal vector using the plane normal
		Vector3d edgeNormal = (d21.cross(planeNormal)).normalized();

		// Get the dist to the edge
		double dist = (p1p - q1).dot(edgeNormal);;;
		maxDist = max(dist, maxDist);
	}
	
	// If maximum distance is more than threshold -> out of poly
	if(maxDist > EDGE_DIST) return false;
	return true;
}

/* ******************************************************************************************** */
void chatterCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {

	// Create the camera transform in the world frame
	Quaternion <double> camQ (-0.340, 0.790, -0.469, 0.202);
	Matrix3d camR = camQ.toRotationMatrix();
  Vector3d camT (0.000, -0.700, 1.400);
	cout << camR << endl;

	// Get the polygon data
	vector < vector <Vector3d> > polygons;
	for(int i = 0; i < msg->markers.size(); i++) {

		// Check if this marker contains polygon data
		const visualization_msgs::Marker& m = msg->markers[i];
		if(m.ns.compare("table_fits/polygon") == 1) {

			// Save the list of points 
			vector <Vector3d> poly;
			cout  << "0 0 0" << endl;
			for(int j = 0; j < m.points.size(); j++) {
				Vector3d loc = Vector3d(m.points[j].x, m.points[j].y, m.points[j].z);
				Vector3d worldP = camR * loc + camT;
				cout <<  worldP.transpose() << endl;
				poly.push_back(worldP);
			}
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

	// For each polygon, create a plane
	set <int> connections;
	for(int poly1 = 0; poly1 < polygons.size(); poly1++) {
		Vector4d& plane1 = planes[poly1];
		for(int poly2 = 0; poly2 < polygons.size(); poly2++) {
			Vector4d& plane2 = planes[poly2];

			// Skip if the same polygons
			if(poly1 == poly2) continue;

			// Skip if these polygons are already established to be connected
			// (Since two planes can not intersect in more than one line)
			int conIdx = min(poly1,poly2) * polygons.size() + max(poly1,poly2);
			if(connections.find(conIdx) != connections.end()) continue;

			// Traverse the edges of first polygon
			for(int vert1 = 0; vert1 < (polygons[poly1].size()-1); vert1++) {
			
				// Get the edge points
				Vector3d p1 = polygons[poly1][vert1];
				Vector3d p2 = polygons[poly1][vert1 + 1];

				// Check if both points lie on the plane of the polygon
				double dist1 = p1(0) * plane2(0) + p1(1) * plane2(1) + p1(2) * plane2(2) + plane2(3);
				double dist2 = p2(0) * plane2(0) + p2(1) * plane2(1) + p2(2) * plane2(2) + plane2(3);
				bool withinPlaneDist1 = fabs(dist1) < PLANE_DIST;
				bool withinPlaneDist2 = fabs(dist2) < PLANE_DIST;
				if(!(withinPlaneDist1 && withinPlaneDist2)) continue;

				// Check if the projection of at least one of the points (p1,p2) to the poly2 plane
				// lies in the polygon poly2
				bool withinPoly1 = checkWithinPoly(p1, plane2, polygons[poly2]);
				bool withinPoly2 = checkWithinPoly(p2, plane2, polygons[poly2]);
				if(!(withinPoly1 || withinPoly2)) continue;
				
				// Save the points
				printf("%d and %d, for points %c and %c\n", poly1, poly2, 'a' + vert1, 'a' + vert1 + 1);
				connections.insert(conIdx);
			}
		}
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
