/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 


#include <ecto_rbo_pcl/common.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

namespace ecto_rbo_pcl
{

using namespace ecto;
using ecto::pcl::PointCloud;
using ecto::pcl::Indices;
using ecto::pcl::xyz_cloud_variant_t;
using ecto::tendrils;
using ecto::spore;

struct CropBox
{
    static void declare_params(tendrils& params)
    {
        UnalignedAffine3f default_transform = Eigen::Affine3f::Identity();
        UnalignedVector3f default_translation(0.0, 0.0, 0.0);
//		UnalignedVector3f default_rotation(::pcl::deg2rad(-90.0f), 0.0, 0.0);
        UnalignedVector3f default_rotation(0.0, 0.0, 0.0);
        UnalignedVector4f default_min(-1.5, -1.5, -1.5, 0.0);
        UnalignedVector4f default_max(1.5, 1.5, 1.5, 0.0);

        params.declare<UnalignedAffine3f>("default_transform", "Transform.", default_transform);
        params.declare<UnalignedVector3f>("default_translation", "Translation.", default_translation);
        params.declare<UnalignedVector3f>("default_rotation", "Rotation.", default_rotation);
        params.declare<UnalignedVector4f>("default_min", "Minimum value for the filter.", default_min);
        params.declare<UnalignedVector4f>("default_max", "Maximum value for the filter.", default_max);
        params.declare<bool>("keep_organized", "Should the output be organized?", true);
        
        params.declare<bool>("publish_rviz_markers", "Should the output be published for visualization?", false);
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<UnalignedAffine3f>(&CropBox::transform_, "transform", "The reference frame of the box.").required(false);
        inputs.declare<UnalignedVector3f>(&CropBox::translation_, "translation", "The translation of the box w.r.t. the reference frame.").required(false);
        inputs.declare<UnalignedVector3f>(&CropBox::rotation_, "rotation", "The rotation of the box w.r.t. the reference frame.").required(false);
        inputs.declare<UnalignedVector4f>(&CropBox::min_, "min", "The min of the box.").required(false);
        inputs.declare<UnalignedVector4f>(&CropBox::max_, "max", "The max of the box.").required(false);
        
        outputs.declare<PointCloud>("output", "Filtered Cloud.");
        outputs.declare<ecto::pcl::Clusters>("indices", "Point Cloud clusters.");
        outputs.declare< ::pcl::ModelCoefficientsConstPtr>("box", "Box used to crop points.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        output_ = outputs["output"];
        indices_ = outputs["indices"];
        box_ = outputs["box"];

        default_transform_ = params["default_transform"];
        default_rotation_ = params["default_rotation"];
        default_translation_ = params["default_translation"];
        default_min_ = params["default_min"];
        default_max_ = params["default_max"];
        keep_organized_ = params["keep_organized"];
        publish_rviz_markers_ = params["publish_rviz_markers"];

        *transform_ = *default_transform_;
        *translation_ = *default_translation_;
        *rotation_ = *default_rotation_;
        *max_ = *default_max_;
        *min_ = *default_min_;
    }

	void publishRVizMarker(const std::string& frame_id)
    {
        static tf::TransformBroadcaster tf_broadcaster;
        tf::Transform bt;
        tf::transformEigenToTF((Eigen::Affine3d) transform_->cast<double>().inverse(), bt);
//	        tf::TransformEigenToTF(transform_->cast<double>(), bt);
        bt.setRotation(bt.getRotation().normalized());
        geometry_msgs::TransformStamped msg;
        tf::transformTFToMsg(bt, msg.transform);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;
        msg.child_frame_id = "crop_box";
        tf_broadcaster.sendTransform(msg);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/crop_box";
        //marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time(0);
        marker.id = 1;
        marker.ns = "crop_box";
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        Eigen::Quaternionf q = Eigen::AngleAxisf((*rotation_)[0], Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf((*rotation_)[1], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf((*rotation_)[2], Eigen::Vector3f::UnitZ());
        
        if (max_.user_supplied() && min_.user_supplied())
        {
            marker.pose.position.x = (*translation_)[0] + ((*max_)[0] - ((*max_)[0] - (*min_)[0]) / 2.0);
            marker.pose.position.y = (*translation_)[1] + ((*max_)[1] - ((*max_)[1] - (*min_)[1]) / 2.0);
            marker.pose.position.z = (*translation_)[2] + ((*max_)[2] - ((*max_)[2] - (*min_)[2]) / 2.0);
            marker.scale.x = (*max_)[0] - (*min_)[0];
            marker.scale.y = (*max_)[1] - (*min_)[1];
            marker.scale.z = (*max_)[2] - (*min_)[2];
        }
        else
        {
            marker.pose.position.x = (*translation_)[0] + ((*default_max_)[0] - ((*default_max_)[0] - (*default_min_)[0]) / 2.0);
            marker.pose.position.y = (*translation_)[1] + ((*default_max_)[1] - ((*default_max_)[1] - (*default_min_)[1]) / 2.0);
            marker.pose.position.z = (*translation_)[2] + ((*default_max_)[2] - ((*default_max_)[2] - (*default_min_)[2]) / 2.0);
            marker.scale.x = (*default_max_)[0] - (*default_min_)[0];
            marker.scale.y = (*default_max_)[1] - (*default_min_)[1];
            marker.scale.z = (*default_max_)[2] - (*default_min_)[2];
        }
        //ROS_INFO("marker: %f %f %f", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
        
        marker.pose.orientation.w = q.w();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        
        static ros::NodeHandle nh;
        static ros::Publisher marker_publisher = nh.advertise<visualization_msgs::Marker>("/rgdbtest/crop_box", 1, true);
        marker_publisher.publish(marker);
	}

	void getEulerAngles(const Eigen::Affine3f& t, float& roll, float& pitch,
			float& yaw) {
		roll = atan2f(t(2, 1), t(2, 2));
		pitch = asinf(-t(2, 0));
		yaw = atan2f(t(1, 0), t(0, 0));
	}

    void transformBox(::pcl::ModelCoefficients& box, const Eigen::Affine3f& transf)
    {
        size_t size_3d = box.values.size() / 3;
        
        for (size_t i = 0; i < size_3d; ++i)
        {
            int j = i * 3;
            Eigen::Vector3f pt(box.values[j], box.values[j + 1], box.values[j + 2]);
            Eigen::Vector3f pt_prime(transf * pt);
            box.values[j] = pt_prime[0];
            box.values[j + 1] = pt_prime[1];
            box.values[j + 2] = pt_prime[2];
        }
    }

    template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
            boost::shared_ptr<const ::pcl::PointCloud<Point> >& input) {
//        // transform cloud
//        typename ::pcl::PointCloud<Point>::Ptr aligned_cloud(new typename ::pcl::PointCloud<Point>);
//        ::pcl::transformPointCloud(*input, *aligned_cloud, transform.inverse());
//
//        // get min max
//        Eigen::Vector4f min_pt(*xyz_min_);
//        Eigen::Vector4f max_pt(*xyz_max_);
//        ::pcl::getMinMax3D(*aligned_cloud, min_pt, max_pt);

//        max_pt += Eigen::Vector4f(0.8, 0, 0, 1);
//        min_pt -= Eigen::Vector4f(1.2, 0, 0, 1);
//        Eigen::Translation3f offset(0.5, 0, 0);
//        transform *= offset;

		//ROS_INFO("Incoming transform: %f %f %f", transform_->translation()[0], transform_->translation()[1], transform_->translation()[2]);


        (*transform_) = transform_->inverse();

        ::pcl::CropBox<Point> crop_filter;
        crop_filter.setTransform(*transform_);
        crop_filter.setRotation(*rotation_);
        crop_filter.setTranslation(*translation_);
        
        if (min_.user_supplied())
            crop_filter.setMin(*min_);
        else
            crop_filter.setMin(*default_min_);
        
        if (max_.user_supplied())
            crop_filter.setMax(*max_);
        else
            crop_filter.setMax(*default_max_);
        
        crop_filter.setKeepOrganized(*keep_organized_);
        crop_filter.setInputCloud(input);
//        Eigen::Vector3f euler_angles;
//        getEulerAngles(transform, euler_angles[0], euler_angles[1], euler_angles[2]);
//        filter.setRotation(euler_angles);

//        std::cout << "t = " << (*xyz_centroid_)[0] << " " << (*xyz_centroid_)[1] << " " << (*xyz_centroid_)[2] << std::endl;
//        transform = transform * transform.inverse();
//        std::cout << transform(0,0) << "\t" << transform(0,1) << "\t" << transform(0,2) << "\t" << transform(0,3) << std::endl;
//        std::cout << transform(1,0) << "\t" << transform(1,1) << "\t" << transform(1,2) << "\t" << transform(1,3) << std::endl;
//        std::cout << transform(2,0) << "\t" << transform(2,1) << "\t" << transform(2,2) << "\t" << transform(2,3) << std::endl;
//        std::cout << transform(3,0) << "\t" << transform(3,1) << "\t" << transform(3,2) << "\t" << transform(3,3) << std::endl;

//        filter.setMax(Eigen::Vector4f((*eigen_values_)[0], (*eigen_values_)[1], (*eigen_values_)[2], 1));
//        filter.setMin(Eigen::Vector4f(-(*eigen_values_)[0], -(*eigen_values_)[1], -(*eigen_values_)[2], 1));

        // set box coefficients
        // the 10 model coefficients (Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
        ::pcl::ModelCoefficientsPtr box(new ::pcl::ModelCoefficients);
        box->values.resize(10);
        ::Eigen::Affine3f my_transform_ = transform_->inverse();
        ::Eigen::Quaternionf box_quaternion(my_transform_.rotation());
        ::Eigen::Vector3f box_size = (*default_max_ - *default_min_).topRows(3);
        ::Eigen::Vector3f box_center = (my_transform_) * (*translation_ + default_min_->topRows(3) + 0.5 * box_size);
        
        if (min_.user_supplied() && max_.user_supplied())
        {
            box_size = (*max_ - *min_).topRows(3);
            box_center = (my_transform_) * (*translation_ + min_->topRows(3) + 0.5 * box_size);
        }
//        ::Eigen::Vector3f box_center = transform_->translation();
        box->values[0] = box_center[0];
        box->values[1] = box_center[1];
        box->values[2] = box_center[2];
        box->values[3] = box_quaternion.x();
        box->values[4] = box_quaternion.y();
        box->values[5] = box_quaternion.z();
        box->values[6] = box_quaternion.w();
        box->values[7] = box_size[0];
        box->values[8] = box_size[1];
        box->values[9] = box_size[2];
        //transformBox(*box, transform);

        (*box_) = box;
        
        if (*publish_rviz_markers_)
            publishRVizMarker(input->header.frame_id);
        
        Indices::Ptr inliers ( new Indices() );
        crop_filter.filter(inliers->indices);
        
        indices_->clear();
        indices_->push_back(*inliers);
        
        ::pcl::ExtractIndices<Point> filter;
        filter.setIndices(inliers);
        filter.setKeepOrganized(true);
        filter.setInputCloud(input);
        
        ROS_INFO("After cropping: %zu", inliers->indices.size());
        
        typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);
        cloud->header = input->header;
        
        filter.filter(*cloud);
        *output_ = xyz_cloud_variant_t(cloud);
        
        if (*publish_rviz_markers_)
        {
            // publish inliers to debug
            sensor_msgs::PointCloud2 color_msg;
            ::pcl::toROSMsg(*cloud, color_msg);
            color_msg.header.frame_id = cloud->header.frame_id;
            color_msg.header.stamp = ::ros::Time::now();
            static ::ros::NodeHandle nh;
            static ::ros::Publisher segmentation_publisher = nh.advertise< sensor_msgs::PointCloud2 > ("/crop_box/inliers", 1);
            segmentation_publisher.publish(color_msg);
        }
        
        return ecto::OK;
    }

    spore<PointCloud> output_;
    spore<ecto::pcl::Clusters> indices_;
    spore< ::pcl::ModelCoefficientsConstPtr> box_;

    spore<UnalignedAffine3f> transform_;
    spore<UnalignedVector3f> translation_;
    spore<UnalignedVector3f> rotation_;
    spore<UnalignedVector4f> max_;
    spore<UnalignedVector4f> min_;
    
    spore<UnalignedAffine3f> default_transform_;
    spore<UnalignedVector3f> default_translation_;
    spore<UnalignedVector3f> default_rotation_;
    spore<UnalignedVector4f> default_max_;
    spore<UnalignedVector4f> default_min_;
    spore<bool> keep_organized_;
    spore<bool> publish_rviz_markers_;
};

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::CropBox>, "CropBox", "Crops a box out of the input cloud, given a reference frame, a transform and a size vector.");
