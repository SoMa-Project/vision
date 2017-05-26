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
	// cout << camR << endl;

	// Get the polygon data
	vector < vector <Vector3d> > polygons;
	vector <Vector3d> means;
	for(int i = 0; i < msg->markers.size(); i++) {

		// Check if this marker contains polygon data
		const visualization_msgs::Marker& m = msg->markers[i];
		if(m.ns.compare("table_fits/polygon") == 1) {

			// Save the list of points 
			vector <Vector3d> poly;
			// cout  << "0 0 0" << endl;
			Vector3d mean (0,0,0);
			for(int j = 0; j < m.points.size(); j++) {
				Vector3d loc = Vector3d(m.points[j].x, m.points[j].y, m.points[j].z);
				Vector3d worldP = camR * loc + camT;
				// cout <<  worldP.transpose() << endl;
				poly.push_back(worldP);
				if(j < (m.points.size() - 1)) mean += worldP;
			}
			mean /= (m.points.size() - 1);
			means.push_back(mean);
			polygons.push_back(poly);
		}
	}

	// Compute the plane parameters for each polygon
	// cout  << "0 0 0" << endl;
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
		// cout << n.transpose() << endl;
	}

	// Print the header
	printf("#VRML V2.0 utf8\n"
		"#IndexedFaceSet polygons\n"
		"Transform {\n"
  	"\ttranslation 0.84999996 0 -0.69999998\n"
		"\tchildren [\n");

	// For each polygon, create a shape in wrl file
	set <int> connections;
	for(int poly = 0; poly < polygons.size(); poly++) {
	//for(int poly = 0; poly < 1; poly++) {

		// print the header
		printf("\t\tTransform {\n"
			"\t\t\ttranslation %lf %lf %lf\n"
      "\t\t\tchildren \n"
			"\t\t\tShape {\n"
			"\t\t\t\tappearance Appearance{\n"
			"\t\t\t\t\tmaterial Material { \n"
			"\t\t\t\t\t\tdiffuseColor %lf %lf %lf\n"
			"\t\t\t\t\t}\n"
			"\t\t\t\t}\n"
			"\t\t\t\tgeometry IndexedFaceSet {\n"
			"\t\t\t\t\tcoord Coordinate {\n"
			"\t\t\t\t\t\tpoint [\n", 
				means[poly](0), means[poly](1), means[poly](2),
				1.0, 0.0, 0.0); 

		// Print the top side of the polyhedra
		printf("\t\t\t\t\t\t\t# top\n");
		for(int j = 0; j < (polygons[poly].size() - 1); j++) {
			Vector3d p = polygons[poly][j] - means[poly];
			printf("\t\t\t\t\t\t\t%lf %lf %lf,\n", p(0), p(1), p(2));
		}

		// Print the bottom side of the polyhedra
		printf("\t\t\t\t\t\t\t# bottom\n");
		for(int j = 0; j < (polygons[poly].size() - 1); j++) {
			Vector3d p = polygons[poly][j] - means[poly];
			Vector3d n = planes[poly].block<3,1>(0,0);
			Vector3d p2 = p + n * 0.01;
			printf("\t\t\t\t\t\t\t%lf %lf %lf,\n", p2(0), p2(1), p2(2));
		}
		printf("\t\t\t\t\t\t]\n\t\t\t\t\t}\n");

		// Create the face indices
		int nV = (polygons[poly].size() - 1);
		printf("\t\t\t\t\tcoordIndex [\n\t\t\t\t\t\t");		
		// ... top face (note the backwards)
		for(int j = 0; j < nV; j++) printf("%d, ", (nV-j)%nV);
		printf("0, -1, \n\t\t\t\t\t\t");
		// ... bottom face
		for(int j = 0; j < nV; j++) printf("%d, ", j + nV);
		printf("%d, -1, \n", nV);
		// ... side faces
		for(int j = 0; j < nV; j++) 
			printf("\t\t\t\t\t\t%d, %d, %d, %d, %d, -1\n", j, ((j+1)%nV), ((j+1)%nV)+nV, (j+nV), j);
		printf("\t\t\t\t\t]\n");
		
		// Wrap up
		printf("\t\t\t\t}\n");
		printf("\t\t\t}\n");
		printf("\t\t}\n");
	}

	printf("\t]\n}\n");

  // ROS_INFO("I heard! %d\n", polygons.size());
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
