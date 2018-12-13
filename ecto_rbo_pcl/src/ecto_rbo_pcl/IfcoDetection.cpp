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
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ifco_pose_estimator/ifco_pose.h"

#include <fstream>

using namespace ecto;
using namespace ecto::pcl;

#define pc(x) std::cout << #x << ": " << (x) << std::endl;
#define ps(x) std::cout << #x << std::endl;
#define pv(x) std::cout << #x << ": " << (x).transpose() << std::endl;

namespace ecto_rbo_pcl
{

typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;
typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

// ======================================================================================================================
// ======================================================================================================================
struct IfcoDetection
{
    ros::NodeHandle nh_;

    // needed for icp detection
    tf::TransformBroadcaster br;
    ros::ServiceClient client = nh_.serviceClient<ifco_pose_estimator::ifco_pose>("ifco_pose");
    ifco_pose_estimator::ifco_pose srv;

    // needed to read static transform
    tf::TransformListener tf_listener_;

    // inputs
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_planes_biggest_;
    spore<double> ifco_length_;
    spore<double> ifco_width_;
    spore<double> ifco_height_;

    // parameters
    spore<double> tableDist_;
    spore<int> plane_id_;
    spore<float> icp_offset_;

    // outputs
    spore<UnalignedAffine3f> ifco_wall_0_transform_;
    spore<UnalignedAffine3f> ifco_wall_1_transform_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > ifco_polygons_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > ifco_planes_;
    spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > ifco_planes_biggest_;
    spore<UnalignedAffine3f> ifco_transform_;

    ::ros::Time last_marker_message_;

    // ======================================================================================================================
    static void declare_params(ecto::tendrils& params)
    {
        params.declare<double>("tableDist", "Distance of a bounded plane to the biggest bounded plane (i.e. table)", 0.0);
        params.declare<int>("plane_id", "Id/Numerator of the plane that is considered as main plane out of all bounded_planes.", 0.0);
        params.declare<float>("icp_offset", "Offset to add on z-Axis of ICP detected IFCO frame", 0.0);
    }

    // ======================================================================================================================
    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes", "Rectangular 3D Planes.");
        inputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_planes_biggest", "Biggest Rectangular 3D Planes.");
        inputs.declare<double>("ifco_length", "Size of the long IFCO edge", 0.0);
        inputs.declare<double>("ifco_width", "Size of the short IFCO edge", 0.0);
        inputs.declare<double>("ifco_height", "Depth of the ifco", 0.0);


        outputs.declare<UnalignedAffine3f>("ifco_wall_0_transform", "Transform of the biggest IFCO wall.");
        outputs.declare<UnalignedAffine3f>("ifco_wall_1_transform", "Transform of the perpendicular IFCO wall.");
        outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("ifco_polygons", "Polygons of the IFCO.");
        outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("ifco_planes", "Planes of the IFCO.");
        outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("ifco_planes_biggest", "Bottom plane of the IFCO.");
        outputs.declare<UnalignedAffine3f>("ifco_transform", "Transform of the IFCO.");
    }

    // ======================================================================================================================
    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        // inputs
        bounded_planes_ = inputs["bounded_planes"];
        bounded_planes_biggest_ = inputs["bounded_planes_biggest"];
        ifco_length_ = inputs["ifco_length"];
        ifco_width_ = inputs["ifco_width"];
        ifco_height_ = inputs["ifco_height"];

        // parameters
        tableDist_ = params["tableDist"];
        plane_id_ = params["plane_id"];
        icp_offset_ = params["icp_offset"];

        // outputs
        ifco_wall_0_transform_ = outputs["ifco_wall_0_transform"];
        ifco_wall_1_transform_ = outputs["ifco_wall_1_transform"];
        ifco_polygons_ = outputs["ifco_polygons"];
        ifco_planes_ = outputs["ifco_planes"];
        ifco_planes_biggest_ = outputs["ifco_planes_biggest"];
        ifco_transform_ = outputs["ifco_transform"];

        ros::Time::init();
        last_marker_message_ = ::ros::Time::now();
    }

    // ==========================================================================================
    ::pcl::ModelCoefficients::Ptr createPolygon(tf::Vector3 origin,
                                                tf::Vector3 principal, tf::Vector3 third_axis, double width, double height) {
        ::pcl::ModelCoefficients::Ptr polygon(new ::pcl::ModelCoefficients());
        polygon->values.resize(12);
        tf::Vector3 p1 = origin + 0.5 * width * principal + 0.5 * height * third_axis;
        tf::Vector3 p2 = origin + 0.5 * width * principal - 0.5 * height * third_axis;
        tf::Vector3 p3 = origin - 0.5 * width * principal - 0.5 * height * third_axis;
        tf::Vector3 p4 = origin - 0.5 * width * principal + 0.5 * height * third_axis;
        polygon->values[0] = p1[0]; polygon->values[1] = p1[1]; polygon->values[2] = p1[2];
        polygon->values[3] = p2[0]; polygon->values[4] = p2[1]; polygon->values[5] = p2[2];
        polygon->values[6] = p3[0]; polygon->values[7] = p3[1]; polygon->values[8] = p3[2];
        polygon->values[9] = p4[0]; polygon->values[10] = p4[1]; polygon->values[11] = p4[2];
        return polygon;
    }

    // ==========================================================================================
    ::pcl::ModelCoefficients::Ptr createBounded(tf::Vector3 origin,
                                                tf::Vector3 normal, tf::Vector3 principal, double width) {
        ::pcl::ModelCoefficients::Ptr bounded_model(new ::pcl::ModelCoefficients());
        bounded_model->values.resize(10);
        bounded_model->values[0] = origin[0];
        bounded_model->values[1] = origin[1];
        bounded_model->values[2] = origin[2];
        bounded_model->values[3] = normal[0];
        bounded_model->values[4] = normal[1];
        bounded_model->values[5] = normal[2];
        bounded_model->values[6] = principal[0];
        bounded_model->values[7] = principal[1];
        bounded_model->values[8] = principal[2];
        bounded_model->values[9] = width;
        return bounded_model;
    }

    // ==========================================================================================
    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {

        tf::Vector3 ifcoCenter;
        tf::Vector3 wall0normal;
        tf::Vector3 wall0origin;
        tf::Vector3 wall1normal;
        tf::Vector3 wall1origin;
        tf::Vector3 wall0originProj;
        tf::Vector3 wall0normalProj;
        tf::Vector3 wall1originProj;
        tf::Vector3 wall1normalProj;

        tf::Vector3 biggestOrigin;
        tf::Vector3 biggestNormal;
        tf::Vector3 biggestPrincipalAxis;
        tf::Vector3 third_axis;

        Eigen::Matrix3f ifcoRotation;
        Eigen::Translation3f ifcoCenter_eigen;

        UnalignedAffine3f transform;
        // Initialize the transform to something
        (*ifco_wall_0_transform_) = UnalignedAffine3f::Identity();
        (*ifco_wall_1_transform_) = UnalignedAffine3f::Identity();
        (*ifco_transform_) = UnalignedAffine3f::Identity();

        // get rosparam that decides on ifco detection method
        enum DetectionMethod { backup=1, simplestatic=2, icp=3 };
        int detection_method_ = 1;
        nh_.param("detection_method", detection_method_, 1); // default is backup ifco detection: use ecto vision to compute it
        const DetectionMethod detection_method = (DetectionMethod) detection_method_;

        // get rosparam that decides if estimated ifco pose should be saved to file
        int save_estimated_ifco_pose_ = 0;
        nh_.param("save_estimated_ifco_pose", save_estimated_ifco_pose_, 0);
        const bool save_estimated_ifco_pose = save_estimated_ifco_pose_ != 0;


        // use an external method to retrieve the ifco transform
        if(detection_method == simplestatic || detection_method == icp)
        {
            tf::Transform cam_to_ifco;

            // icp method is selected
            // compute ifco transform and broadcast it
            if(detection_method == icp)
            {
                srv.request.max_tries = 10;
                srv.request.max_fitness = 0.008;
                srv.request.publish_ifco = true;

                if (client.call(srv))
                {
                    ROS_INFO("Service was called");
                    ROS_INFO("Fitness value was %f", srv.response.fitness);
                    tf::poseMsgToTF(srv.response.pose, cam_to_ifco);
                }
                else
                {
                    ROS_ERROR("Failed to call service");
                }

            }

            // access static transform (which comes either from the service call above or is sent otherwise)
            try
            {
                tf::StampedTransform transform_stamped;
                Eigen::Matrix3f ifco_rotation_icp;
                Eigen::Matrix3f ifco_rotation_wallconventions;

                if (detection_method == simplestatic)
                {
                    tf_listener_.lookupTransform("camera_rgb_optical_frame", "ifco_static", ros::Time(0), transform_stamped);
                }
                else
                {
                    transform_stamped = tf::StampedTransform(cam_to_ifco, ros::Time::now(), "camera_rgb_optical_frame", "ifco_icp");
                    br.sendTransform(transform_stamped);
                    // this transform needs to be applied on the original icp to meet our conventions
                    ifco_rotation_icp <<  1 , 0, 0,
                            0 , -1, 0,
                            0 ,0, -1;

                    ifco_rotation_wallconventions <<  -1 , 0, 0,
                            0 , -1, 0,
                            0 ,0, 1;
                }
                // get ifco center
                ifcoCenter = transform_stamped.getOrigin();
                // convert ifco transform from tf::Quaternion to Eigen::Quaternion then Eigen::Matrix3f
                tf::Quaternion ifcoRotation_tf = transform_stamped.getRotation();
                Eigen::Quaterniond ifcoRotation_eigen;
                tf::quaternionTFToEigen (ifcoRotation_tf, ifcoRotation_eigen);
                Eigen::Matrix3d ifcoRotation_ = ifcoRotation_eigen.toRotationMatrix();
                ifcoRotation = ifcoRotation_.cast<float>();

                if (detection_method == icp)
                {
                    // ifco rotation axis needs to be flipped to satisfy our conventions
                    ifcoRotation = ifcoRotation * ifco_rotation_icp;
                    ifcoRotation = ifcoRotation * ifco_rotation_wallconventions;
                }
                float offset = ifcoCenter.getZ() + (*icp_offset_);
                ifcoCenter_eigen = Eigen::Translation3f(ifcoCenter.getX(), ifcoCenter.getY(), offset);
                transform = ifcoCenter_eigen * ifcoRotation;
                (*ifco_transform_) = transform;

                if(save_estimated_ifco_pose && detection_method == icp) {
                    tf::Quaternion rotation_tf;
                    tf::quaternionEigenToTF(Eigen::Quaterniond(ifcoRotation.cast<double>()),rotation_tf);
                    writeEstimatedIfcoPose2File(ifcoCenter, rotation_tf);
                }
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }


            // rotations of wall frame 0/1 according to convention (z: towards ifcoCenter, y: along ifcoCenter z-axis, x: along wall)
            // generate wall 0 -> long wall
            UnalignedAffine3f transform_wall0;
            Eigen::Translation3f wall_0_Center = Eigen::Translation3f(0, 0, - (*ifco_width_) * 0.5);

            Eigen::Matrix3f wall_0_rotation;
            wall_0_rotation <<  1 , 0, 0,
                    0 , 0, 1,
                    0 ,1, 0;
            Eigen::Matrix3f ifco_rotated_for0;
            ifco_rotated_for0 <<  -ifcoRotation(0,0), ifcoRotation(0,2), ifcoRotation(0,1),
                    -ifcoRotation(1,0), ifcoRotation(1,2), ifcoRotation(1,1),
                    -ifcoRotation(2,0), ifcoRotation(2,2), ifcoRotation(2,1);

            UnalignedAffine3f transform0 = ifcoCenter_eigen * ifco_rotated_for0 ;

            transform_wall0 = transform0 * wall_0_Center;


            // generate wall 1 -> short wall
            UnalignedAffine3f transform_wall1;
            Eigen::Translation3f wall_1_Center = Eigen::Translation3f(0, 0, -(*ifco_length_) * 0.5);

            Eigen::Matrix3f wall_1_rotation;
            wall_1_rotation <<  0 , 0, -1,
                    -1 , 0, 0,
                    0 ,1, 0;

            Eigen::Matrix3f ifco_rotated_for1;
            ifco_rotated_for1 <<  -ifcoRotation(0,1), ifcoRotation(0,2), -ifcoRotation(0,0),
                    -ifcoRotation(1,1), ifcoRotation(1,2), -ifcoRotation(1,0),
                    -ifcoRotation(2,1), ifcoRotation(2,2), -ifcoRotation(2,0);

            UnalignedAffine3f transform1 = ifcoCenter_eigen * ifco_rotated_for1 ;
            transform_wall1 =  transform1 * wall_1_Center ;

            (*ifco_wall_0_transform_) = transform_wall0;
            (*ifco_wall_1_transform_) = transform_wall1;


            // overwrite other vector stuff
            biggestNormal = tf::Vector3(ifcoRotation(0,2), ifcoRotation(1,2), ifcoRotation(2,2));
            biggestPrincipalAxis = tf::Vector3(ifcoRotation(0,1), ifcoRotation(1,1), ifcoRotation(2,1));
            biggestOrigin = ifcoCenter;
            wall0origin = tf::Vector3(wall_0_Center.x(), wall_0_Center.y(), wall_0_Center.z());
            wall1origin = tf::Vector3(wall_1_Center.x(), wall_1_Center.y(), wall_1_Center.z());
            wall0normal = tf::Vector3(ifco_rotated_for0(0,2), ifco_rotated_for0(1,2), ifco_rotated_for0(2,2));
            wall1normal = tf::Vector3(ifco_rotated_for1(0,2), ifco_rotated_for1(1,2), ifco_rotated_for1(2,2));


            ifco_planes_->clear();
            ifco_planes_biggest_->clear();
            ifco_polygons_->clear();


            tf::Vector3 wall0originB = ifcoCenter - 0.5 * ((*ifco_width_)  * wall0normal)+ 0.5 * ((*ifco_height_) * biggestNormal);
            tf::Vector3 wall1originB = ifcoCenter - 0.5 * ((*ifco_length_) * wall1normal)+ 0.5 * ((*ifco_height_) * biggestNormal);
            tf::Vector3 wall2originB = ifcoCenter + 0.5 * ((*ifco_width_)  * wall0normal)+ 0.5 * ((*ifco_height_) * biggestNormal);
            tf::Vector3 wall3originB = ifcoCenter + 0.5 * ((*ifco_length_) * wall1normal)+ 0.5 * ((*ifco_height_) * biggestNormal);


            // Create the bounded models for the primitives
            ifco_planes_->push_back(createBounded(wall0originB, wall0normal, -(wall0normal.cross(biggestNormal)).normalized(), (*ifco_height_)));
            ifco_planes_->push_back(createBounded(wall1originB, wall1normal, -(wall1normal.cross(biggestNormal)).normalized(), (*ifco_height_)));
            ifco_planes_->push_back(createBounded(wall2originB, -wall0normal, -wall1normal, (*ifco_height_)));
            ifco_planes_->push_back(createBounded(wall3originB, -wall1normal,  wall0normal, (*ifco_height_)));

            ifco_planes_->push_back(createBounded(ifcoCenter,-biggestNormal, (*ifco_length_) *biggestPrincipalAxis, (*ifco_height_)));
            ifco_planes_biggest_->push_back(createBounded(ifcoCenter,-biggestNormal, (*ifco_length_) *biggestPrincipalAxis, (*ifco_height_)));


            // Create the polygons (for wall grasps)
            ifco_polygons_->push_back(createPolygon(wall0originB, wall1normal, biggestNormal, (*ifco_length_), (*ifco_height_)));
            ifco_polygons_->push_back(createPolygon(wall1originB, wall0normal, biggestNormal, (*ifco_width_), (*ifco_height_)));
            ifco_polygons_->push_back(createPolygon(wall2originB, -wall1normal, biggestNormal, (*ifco_length_), (*ifco_height_)));
            ifco_polygons_->push_back(createPolygon(wall3originB, -wall0normal, biggestNormal, (*ifco_width_), (*ifco_height_)));
            ifco_polygons_->push_back(createPolygon(ifcoCenter, wall1normal, wall0normal, (*ifco_length_), (*ifco_width_)));



        }
        // no static ifco available so it will be computed from vision
        else if(detection_method == backup)
        {

            // Get the size of the biggest bounded plane
            std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator biggestIt = bounded_planes_biggest_->begin();
            biggestOrigin = tf::Vector3 ((*biggestIt)->values[0], (*biggestIt)->values[1], (*biggestIt)->values[2]);
            biggestNormal = tf::Vector3 ((*biggestIt)->values[3], (*biggestIt)->values[4], (*biggestIt)->values[5]);
            biggestPrincipalAxis = tf::Vector3 ((*biggestIt)->values[6], (*biggestIt)->values[7], (*biggestIt)->values[8]);
            double biggestSize = biggestPrincipalAxis.length() * (*biggestIt)->values[9];



            // --- PART 1 -----------------------------------------------------------------------
            // Iterate through the bounded planes to find the biggest IFCO wall we can see
            double maxSize = -1;
            ::pcl::ModelCoefficientsConstPtr ifcoWall0It;
            double wall0angle;
            int counter = -1;
            printf("\n\n\n~~~ Looking for wall 1!\n");
            for (std::vector< ::pcl::ModelCoefficientsConstPtr>::iterator it =
                 bounded_planes_->begin(); it != bounded_planes_->end(); ++it) {

                counter++;
                if((*plane_id_ != -1) && counter != *plane_id_) {
                    printf("skipping because %d vs. %d\n", counter, *plane_id_);
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
                if((angle < M_PI_2/2.0) || (angle > 1.5 * M_PI_2)) {
                    pc(angle / M_PI * 180.0);
                    printf("\tnot perpendicular!\n");
                    continue;
                }

                // Check if the principle axis of the bounding box is perpendicular to the table normal
                // Note from Can: Not sure why I wrote this in the first place. Leave it until MS4 in case necessary.
                double angle2 = acos(principal_axis.dot(biggestNormal));
                if(false && (((angle2 < (M_PI_2 - 0.1)) || (angle2 > (M_PI_2 + 0.1))))) {
                    pc(angle2 / M_PI * 180.0);
                    printf("\tslanted!\n");
                    continue;
                }


                // Check if the bounded box is close enough to the biggest plane (i.e. it is on the table)
                double dist = (origin - biggestOrigin).dot(biggestNormal);
                if(fabs(dist) > *tableDist_ || dist > 0) {
                    pc(dist);
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

                    // Make sure that the normal axis is looking towards the middle of the ifco
                    if(wall0normal.dot(biggestOrigin - wall0origin) < 0)
                        wall0normal *= -1;

                    /*
                    // Update the transform for visualization
                    third_axis = wall0normal.cross(principal_axis);
                    Eigen::Matrix3f rotation;
                    rotation << principal_axis.x(), third_axis.x(), wall0normal.x(),
                            principal_axis.y(), third_axis.y(), wall0normal.y(),
                            principal_axis.z(), third_axis.z(), wall0normal.z();
                    UnalignedAffine3f transform = Eigen::Translation3f(origin[0], origin[1], origin[2]) * rotation;
                    (*ifco_wall_0_transform_) = transform;*/
                }
            }

            if(maxSize < 0) {
                ROS_ERROR("Could not find the first wall");
                return QUIT;
            }
            printf("maxSize: %lf\n", maxSize);

            // ---- PART 2 ----------------------------------------------------------------------
            // Find a second wall of the IFCO that is perpendicular to the first wall and the
            // table surface of course
            maxSize = -1;
            ::pcl::ModelCoefficientsConstPtr ifcoWall1It;

            printf("~~~ Looking for wall 2!\n");
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

                    // Make sure that the normal axis is looking towards the middle of the ifco
                    if(wall1normal.dot(wall0origin - wall1origin) < 0)
                        wall1normal *= -1;

                    /*
                    // Update the transform for visualization
                    third_axis = wall1normal.cross(principal_axis);
                    Eigen::Matrix3f rotation;
                    rotation << principal_axis.x(), third_axis.x(), wall1normal.x(),
                            principal_axis.y(), third_axis.y(), wall1normal.y(),
                            principal_axis.z(), third_axis.z(), wall1normal.z();
                    UnalignedAffine3f transform = Eigen::Translation3f(origin[0], origin[1], origin[2]) * rotation;
                    (*ifco_wall_1_transform_) = transform;*/
                }
            }

            if(maxSize < 0) {
                ROS_ERROR("Could not find the second wall");
                return QUIT;
            }

            // Compute the origin and normal projections of the walls to the table surface

            wall0originProj = wall0origin - ((wall0origin - biggestOrigin).dot(biggestNormal)) * biggestNormal;
            wall0normalProj = (wall0normal - (wall0normal.dot(biggestNormal)) * biggestNormal).normalized();
            wall1originProj = wall1origin - ((wall1origin - biggestOrigin).dot(biggestNormal)) * biggestNormal;
            wall1normalProj = (wall1normal - (wall1normal.dot(biggestNormal)) * biggestNormal).normalized();

            // Compute the center of the IFCO on the biggest plane (table) ...
            // ... by projecting wall centers and normals to the table plane

            if(0) {
                ifcoCenter = (wall1originProj - wall0originProj).dot(wall0normalProj) * wall0normalProj + wall0originProj;
            }

            // ... or, first get the corner location, and then using ifco size and long wall information, estimate the center (this should be more stable!)
            else {
                tf::Vector3 wall0inDir = (wall0normalProj.cross(biggestNormal)).normalized();
                double t = (wall0originProj - wall1originProj).dot(wall1normalProj) / (wall0inDir.dot(wall1normalProj));

                // We are not sure of the normal directions so we check both translation of magnitude t
                ifcoCenter = wall0originProj + wall0inDir * t;
                static const double kProjLimit = 0.05;
                if(((ifcoCenter-wall0originProj).dot(wall0normalProj) > kProjLimit) || ((ifcoCenter-wall1originProj).dot(wall1normalProj) > kProjLimit))
                    ifcoCenter =  wall0originProj - wall0inDir * t;

                // Move the ifco center from the corner to the actual center with hardcoded values
                ifcoCenter += (0.19 * wall0normalProj) + (0.27 * wall1normalProj);
            }

            // Compute the orientation
            Eigen::Matrix3f rotation;// = Eigen::Matrix3f::Identity();
            third_axis = wall0normalProj.cross(-biggestNormal);
            rotation << third_axis.x(), wall0normalProj.x(), -biggestNormal.x(),
                    third_axis.y(), wall0normalProj.y(), -biggestNormal.y(),
                    third_axis.z(), wall0normalProj.z(), -biggestNormal.z();

            transform = Eigen::Translation3f(ifcoCenter[0], ifcoCenter[1], ifcoCenter[2]) * rotation;
            (*ifco_transform_) = transform;
            if(save_estimated_ifco_pose) {
                tf::Quaternion rotation_tf;
                tf::quaternionEigenToTF(Eigen::Quaterniond(rotation.cast<double>()),rotation_tf);
                writeEstimatedIfcoPose2File(ifcoCenter, rotation_tf);
            }


            // Create the bounded models for the primitives
            ifco_planes_->clear();
            ifco_planes_biggest_->clear();


            tf::Vector3 wall0originB = ifcoCenter - 0.5 * ((*ifco_width_) * wall0normalProj) - 0.5 * ((*ifco_height_) * biggestNormal);
            tf::Vector3 wall2originB = ifcoCenter + 0.5 * ((*ifco_width_) * wall0normalProj) - 0.5 * ((*ifco_height_) * biggestNormal);
            tf::Vector3 wall1originB = ifcoCenter - 0.5 * ((*ifco_length_) * wall1normalProj) - 0.5 * ((*ifco_height_) * biggestNormal);
            tf::Vector3 wall3originB = ifcoCenter + 0.5 * ((*ifco_length_) * wall1normalProj) - 0.5 * ((*ifco_height_) * biggestNormal);

            // Make sure that the normal axis is looking towards the middle of the ifco
            //we use the center point of the ifco bottom to check the direction of the normal and
            //we use the correct wall normal and the ifco bottom plain normal crossproduct to get the primary axis

            //long width wall check if normal is pointing toward the middle of the ifco
            if(wall0normal.dot(ifcoCenter - wall0originB) > 0)
                ifco_planes_->push_back(createBounded(wall0originB, wall0normal, (wall0normal.cross(biggestNormal)).normalized(), (*ifco_height_)));
            else //if not flip the direction of the normal
                ifco_planes_->push_back(createBounded(wall0originB, -wall0normal, (-wall0normal.cross(biggestNormal)).normalized(), (*ifco_height_)));

            //short width wall
            if(third_axis.dot(ifcoCenter - wall1originB) > 0)
                ifco_planes_->push_back(createBounded(wall1originB, third_axis, (third_axis.cross(biggestNormal)).normalized(), (*ifco_height_)));
            else
                ifco_planes_->push_back(createBounded(wall1originB, -third_axis, (-third_axis.cross(biggestNormal)).normalized(), (*ifco_height_)));

            //long width wall
            if(wall0normal.dot(ifcoCenter - wall2originB) > 0)
                ifco_planes_->push_back(createBounded(wall2originB, wall0normal, (wall0normal.cross(biggestNormal)).normalized(), (*ifco_height_)));
            else
                ifco_planes_->push_back(createBounded(wall2originB, -wall0normal, (-wall0normal.cross(biggestNormal)).normalized(), (*ifco_height_)));

            //short width wall
            if(third_axis.dot(ifcoCenter - wall3originB) > 0)
                ifco_planes_->push_back(createBounded(wall3originB, third_axis, (third_axis.cross(biggestNormal)).normalized(), (*ifco_height_)));
            else
                ifco_planes_->push_back(createBounded(wall3originB, -third_axis, (-third_axis.cross(biggestNormal)).normalized(), (*ifco_height_)));

            ifco_planes_->push_back(createBounded(ifcoCenter,-biggestNormal, (*ifco_length_) *third_axis, (*ifco_width_)));
            ifco_planes_biggest_->push_back(createBounded(ifcoCenter,-biggestNormal, (*ifco_length_) *third_axis, (*ifco_width_)));

            // Create the polygons (for wall grasps)
            ifco_polygons_->clear();
            ifco_polygons_->push_back(
                        createPolygon(wall0originB, wall1normalProj, biggestNormal, (*ifco_length_), (*ifco_height_)));
            ifco_polygons_->push_back(
                        createPolygon(wall1originB, wall0normalProj, biggestNormal, (*ifco_width_), (*ifco_height_)));
            ifco_polygons_->push_back(
                        createPolygon(wall2originB, -wall1normalProj, biggestNormal, (*ifco_length_), (*ifco_height_)));
            ifco_polygons_->push_back(
                        createPolygon(wall3originB, -wall0normalProj, biggestNormal, (*ifco_width_), (*ifco_height_)));
            ifco_polygons_->push_back(
                        createPolygon(ifcoCenter, wall1normalProj, wall0normalProj, (*ifco_length_), (*ifco_width_)));

        }

    }


    // ======================================================================================================================
    void writeEstimatedIfcoPose2File(const tf::Vector3 &ifcoCenter, const tf::Quaternion &ifcoRotation) {
        const std::string path = ros::package::getPath("planner_gui");
        if(path == "") {
            ROS_ERROR("Could not find planner_gui package => can't write launch file to correct location => abort");
            return;
        }

        // Since the ifco frame is expressed in the camera frame and not the base_link we first have to express the
        // transform in the base frame before writing it to the launch file
        tf::Transform ifco2camera(ifcoRotation, ifcoCenter);
        tf::StampedTransform camera2base;
        try {
            tf_listener_.waitForTransform("base_link", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(3.0));
            tf_listener_.lookupTransform("base_link", "camera_rgb_optical_frame", ros::Time(0), camera2base);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s (Did not write ifco pose)",ex.what());
            return;
        }

        const tf::Transform ifco2base = camera2base * ifco2camera;
        const tf::Vector3 center = ifco2base.getOrigin();
        const tf::Quaternion rot = ifco2base.getRotation();

        // start actually writing the file
        std::ofstream transformLaunchFile;
        transformLaunchFile.open((path + "/launch/staticTFforIfco.launch").c_str());

        transformLaunchFile << "<?xml version=\"1.0\"?>\n<launch>\n";
        transformLaunchFile << "<node pkg=\"tf\" type=\"static_transform_publisher\" name=\"ifco_static\" args=\"";
        transformLaunchFile << center.getX() << " " << center.getY() << " " << center.getZ() << " ";
        transformLaunchFile << rot.getX() << " " << rot.getY() << " " << rot.getZ() << " " << rot.getW();
        transformLaunchFile << " base_link ifco_static 100\" />";
        transformLaunchFile << "\n</launch>\n";

        transformLaunchFile.close();

        ROS_INFO("Ifco pose saved to file.");

    }

};

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCellWithNormals<ecto_rbo_pcl::IfcoDetection>, "IfcoDetection", "Detect Ifco based on static tf first priority otherwise compute from vision");
