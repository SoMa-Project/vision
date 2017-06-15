/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pregrasp_msgs/GraspStrategyArray.h>
#include <ecto_rbo_grasping/PoseSet.h>

using namespace ecto;

#define pc(x) std::cout << #x << ": " << (x) << std::endl;
#define ps(x) std::cout << #x << std::endl;

namespace ecto_rbo_grasping
{

typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

// ======================================================================================================================
// ======================================================================================================================
struct IfcoGrasp 
{
		// inputs
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_biggest_;

		// parameters
    ecto::spore<double> tableDist_;
    ecto::spore<int> plane_id_;

		// outputs
    ecto::spore<pregrasp_msgs::GraspStrategyArrayConstPtr> pregrasp_messages_;
    ecto::spore< ::posesets::PoseSetArrayConstPtr> manifolds_;
		spore<UnalignedAffine3f> ifco_transform_;
		spore<UnalignedAffine3f> ifco_wall_0_transform_;
		spore<UnalignedAffine3f> ifco_wall_1_transform_;

    ::ros::Time last_marker_message_;

		// ======================================================================================================================
    static void declare_params(ecto::tendrils& params)
    {
        params.declare<double>("tableDist", "Distance of a bounded plane to the biggest bounded plane (i.e. table)", 0.0);
        params.declare<int>("plane_id", "afasdfof a bounded plane to the biggest bounded plane (i.e. table)", 0.0);
    }

		// ======================================================================================================================
    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes", "Rectangular 3D Planes.");
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes_biggest", "Biggest Rectangular 3D Planes.");
        outputs.declare<pregrasp_msgs::GraspStrategyArrayConstPtr>("pregrasp_messages", "All the grasps that should be used.");
				outputs.declare<UnalignedAffine3f>("ifco_wall_0_transform", "Transform of the biggest IFCO wall.");
				outputs.declare<UnalignedAffine3f>("ifco_wall_1_transform", "Transform of the perpendicular IFCO wall.");
				outputs.declare<UnalignedAffine3f>("ifco_transform", "Transform of the IFCO.");
    }

		// ======================================================================================================================
    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        bounded_planes_ = inputs["bounded_planes"];
        bounded_planes_biggest_ = inputs["bounded_planes_biggest"];

				// parameters
        tableDist_ = params["tableDist"];
        plane_id_ = params["plane_id"];

				// outputs
				ifco_wall_0_transform_ = outputs["ifco_wall_0_transform"];
				ifco_wall_1_transform_ = outputs["ifco_wall_1_transform"];
				ifco_transform_ = outputs["ifco_transform"];
        pregrasp_messages_ = outputs["pregrasp_messages"];

        ros::Time::init();
        last_marker_message_ = ::ros::Time::now();
    }

		// ======================================================================================================================
    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
				// Initialize the pregrasp messages
        pregrasp_msgs::GraspStrategyArrayPtr messages(new ::pregrasp_msgs::GraspStrategyArray());
        messages->header = pcl_conversions::fromPCL(input->header);

				// Get the size of the biggest bounded plane
				std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator biggestIt = bounded_planes_biggest_->begin();
				tf::Vector3 biggestOrigin ((*biggestIt)->values[0], (*biggestIt)->values[1], (*biggestIt)->values[2]);
				tf::Vector3 biggestNormal ((*biggestIt)->values[3], (*biggestIt)->values[4], (*biggestIt)->values[5]);
				tf::Vector3 biggestPrincipalAxis((*biggestIt)->values[6], (*biggestIt)->values[7], (*biggestIt)->values[8]);
				double biggestSize = biggestPrincipalAxis.length() * (*biggestIt)->values[9];
				double biggestRadius = biggestPrincipalAxis.length();

				// Initialize the transform to something
				(*ifco_wall_0_transform_) = UnalignedAffine3f::Identity();
				(*ifco_wall_1_transform_) = UnalignedAffine3f::Identity();
				(*ifco_transform_) = UnalignedAffine3f::Identity();

				// Iterate through the bounded planes to find the biggest IFCO wall we can see
				double maxSize = 0;
				::pcl::ModelCoefficientsConstPtr ifcoWall0It;
				tf::Vector3 wall0normal;
				tf::Vector3 wall0origin;
				double wall0angle;
				int counter = -1;
        for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = bounded_planes_->begin(); it != bounded_planes_->end(); ++it) {

					counter++;
					if((*plane_id_ != -1) && counter != *plane_id_) {
						printf("\n\nskipping because %d vs. %d\n", counter, *plane_id_);
						continue;
					}
					
					// Take the origin, normal, principle axis and plane size from the bounded plane
					// The 9th index is the length of the minor axis of the bounding box
          tf::Vector3 origin((*it)->values[0], (*it)->values[1], (*it)->values[2]);
          tf::Vector3 normal((*it)->values[3], (*it)->values[4], (*it)->values[5]);
          tf::Vector3 principal_axis((*it)->values[6], (*it)->values[7], (*it)->values[8]);
					double size = principal_axis.length() * (*it)->values[9];
          tf::Vector3 plane_size(principal_axis.length(), (*it)->values[9], 0.02);	
          normal.normalize();
          principal_axis.normalize();

					// Check if the bounded plane is perpendicular enough to the biggest plane
					double angle = acos(normal.dot(biggestNormal));
					//pc(angle / M_PI * 180.0);
					if((angle < M_PI_2/2.0) || (angle > 1.5 * M_PI_2)) {
						printf("\tnot perpendicular!\n");
						continue;
					}

					// Check if the principle axis of the bounding box is perpendicular to the table normal
					double angle2 = acos(principal_axis.dot(biggestNormal));
					//pc(angle2 / M_PI * 180.0);
					if((angle2 < (M_PI_2 - 0.1)) || (angle2 > (M_PI_2 + 0.1))) {
						printf("\tslanted!\n");
						continue;
					}


					// Check if the bounded box is close enough to the biggest plane (i.e. it is on the table)
					double dist = (origin - biggestOrigin).dot(biggestNormal);
					//pc(dist);
					if(fabs(dist) > *tableDist_ || dist > 0) {
						printf("\ttoo far from table, skipping\n");
						continue;
					}

					// Get the size of the plane and see if it can be the biggest IFCO plane
					//pc(size);
					if((size < (biggestSize - 1e-3)) && (size > maxSize)) {

						// Update the size
						printf("\tbiggest now (vs. %lf)!\n", maxSize);
						maxSize = size;
						ifcoWall0It = *it;
						wall0normal = normal;
						wall0origin = origin;
						wall0angle = angle2;
					

						// Update the transform for visualization
            tf::Vector3 third_axis = normal.cross(principal_axis);
            Eigen::Matrix3f rotation;
						rotation << principal_axis.x(), third_axis.x(), normal.x(),
                                   principal_axis.y(), third_axis.y(), normal.y(),
                                   principal_axis.z(), third_axis.z(), normal.z();
						UnalignedAffine3f transform = Eigen::Translation3f(origin[0], origin[1], origin[2]) * rotation;
						(*ifco_wall_0_transform_) = transform;
					}
				}
				pc(wall0angle);

				// Find a second wall of the IFCO that is perpendicular to the first wall and the table surface of course
				maxSize = 0;
				::pcl::ModelCoefficientsConstPtr ifcoWall1It;
				tf::Vector3 wall1normal;
				tf::Vector3 wall1origin;
        for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it = bounded_planes_->begin(); it != bounded_planes_->end(); ++it) {

					// Take the origin, normal, principle axis and plane size from the bounded plane
					// The 9th index is the length of the minor axis of the bounding box
          tf::Vector3 origin((*it)->values[0], (*it)->values[1], (*it)->values[2]);
          tf::Vector3 normal((*it)->values[3], (*it)->values[4], (*it)->values[5]);
          tf::Vector3 principal_axis((*it)->values[6], (*it)->values[7], (*it)->values[8]);
					double size = principal_axis.length() * (*it)->values[9];
          tf::Vector3 plane_size(principal_axis.length(), (*it)->values[9], 0.02);	
          normal.normalize();
          principal_axis.normalize();

					// Check if the bounded plane is perpendicular enough to the biggest plane
					double angle = acos(normal.dot(biggestNormal));
					pc(angle / M_PI * 180.0);
					if((angle < M_PI_2/2.0) || (angle > 1.5 * M_PI_2)) {
						printf("\tnot perpendicular to table!\n");
						continue;
					}

					// Check if the bounded plane is perpendicular enough to the first wall
					angle = acos(normal.dot(wall0normal));
					pc(angle / M_PI * 180.0);
					if((angle < M_PI_2/2.0) || (angle > 1.5 * M_PI_2)) {
						printf("\tnot perpendicular to wall0!\n");
						continue;
					}

					// Check if the bounded box is close enough to the biggest plane (i.e. it is on the table)
					double dist = (origin - biggestOrigin).dot(biggestNormal);
					pc(dist);
					if(fabs(dist) > *tableDist_ || dist > 0) {
						printf("\ttoo far from table, skipping\n");
						continue;
					}

					// Check if the bounded box is different than the first wall
					double wallDist = (origin - wall0origin).length();
					if(wallDist < 0.05) {
						printf("\ttoo close to the first wall, skipping\n");
						continue;
					}

					// Get the size of the plane and see if it can be the biggest IFCO plane
					pc(size);
					if((size < (biggestSize - 1e-3)) && (size > maxSize)) {

						// Update the size
						printf("\tbiggest now (vs. %lf)!\n", maxSize);
						maxSize = size;
						ifcoWall1It = *it;
						wall1normal = normal;
						wall1origin = origin;

						// Update the transform for visualization
            tf::Vector3 third_axis = normal.cross(principal_axis);
            Eigen::Matrix3f rotation;
						rotation << principal_axis.x(), third_axis.x(), normal.x(),
                                   principal_axis.y(), third_axis.y(), normal.y(),
                                   principal_axis.z(), third_axis.z(), normal.z();
						UnalignedAffine3f transform = Eigen::Translation3f(origin[0], origin[1], origin[2]) * rotation;
						(*ifco_wall_1_transform_) = transform;
					}
				}

				// Compute the center of the IFCO on the biggest plane (table) by projecting wall centers and normals
				// to the table plane
				tf::Vector3 wall0originProj = wall0origin - ((wall0origin - biggestOrigin).dot(biggestNormal)) * biggestNormal;
				tf::Vector3 wall0normalProj = wall0normal - (wall0normal.dot(biggestNormal)) * biggestNormal;
				tf::Vector3 wall1originProj = wall1origin - ((wall1origin - biggestOrigin).dot(biggestNormal)) * biggestNormal;
				tf::Vector3 wall1normalProj = wall1normal - (wall1normal.dot(biggestNormal)) * biggestNormal;
				tf::Vector3 ifcoCenter = (wall1originProj - wall0originProj).dot(wall0normalProj) * wall0normalProj + wall0originProj;
				Eigen::Matrix3f rotation;// = Eigen::Matrix3f::Identity();
        tf::Vector3 third_axis = wall0normalProj.cross(-biggestNormal);
				rotation << third_axis.x(), wall0normalProj.x(), -biggestNormal.x(),
															 third_axis.y(), wall0normalProj.y(), -biggestNormal.y(),
															 third_axis.z(), wall0normalProj.z(), -biggestNormal.z();
				UnalignedAffine3f transform = Eigen::Translation3f(ifcoCenter[0], ifcoCenter[1], ifcoCenter[2]) * rotation;
				(*ifco_transform_) = transform;




        (*pregrasp_messages_) = messages;
        return OK;
    }
		// ======================================================================================================================
};

}

ECTO_CELL(ecto_rbo_grasping, ecto::pcl::PclCellWithNormals<ecto_rbo_grasping::IfcoGrasp>, "IfcoGrasp", "Finds grasps for IFCO.");
