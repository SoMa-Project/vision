/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_rbo_pcl/common.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace ecto;
using namespace ecto::pcl;

namespace ecto_rbo_pcl
{

struct PlaneFits
{

//  spore<std::vector<std::vector<int> > > clusters_;
  spore<ecto::pcl::Clusters> clusters_;

  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > plane_coefficients_;
  spore<std::vector<double> > fit_quality_;
  spore<ecto::pcl::Clusters> inliers_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_plane_coefficients_;
  spore<std::vector<UnalignedVector4f> > plane_centroids_;
  spore<std::vector<UnalignedVector3f> > plane_sizes_;
  spore<std::vector<UnalignedAffine3f> > plane_transforms_;

  spore<UnalignedVector4f> plane_centroid_biggest_;
  spore<UnalignedAffine3f> plane_transform_biggest_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > plane_coefficients_biggest_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > bounded_plane_coefficients_biggest_;
  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > polygon_biggest_;

  spore<std::vector< ::pcl::ModelCoefficientsConstPtr> > polygons_;

  // parameters
  spore<double> distance_threshold_;
  spore<double> min_inlier_ratio_;
  spore<double> weight_contour_;
  spore<double> min_boxness_;
  spore<double> max_size_;
  spore<double> min_size_;
  spore<double> polygon_approximation_epsilon_;
  
  spore<bool> publish_rviz_markers_;

  int biggest_index;

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<double>("min_inlier_ratio", "Minimum number of inlier points per plane.", 0.5);
    params.declare<double>("distance_threshold", "Maximum mean error a plane fit may have.", 0.05);
    params.declare<double>("min_boxness", "", 0.8);
    params.declare<double>("weight_contour", "", 0.5);
    params.declare<double>("max_size", "", 0.24);
    params.declare<double>("min_size", "", 0.08);
    params.declare<double>("polygon_approximation_epsilon", "Parameter for Douglas-Peucker algorithm that approximates the planar contour with a polygon (in mm).", 50.0);
    
    params.declare<bool>("publish_rviz_markers", "Should the output be published for visualization?", false);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
//    inputs.declare<std::vector<std::vector<int> > >("clusters", "Point Cloud clusters.");
    inputs.declare< ecto::pcl::Clusters >("clusters", "Point Cloud clusters.");

    outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("models", "Plane coefficients");
    outputs.declare<std::vector<double> >("fit_quality", "Goodness of fit.");
    outputs.declare<ecto::pcl::Clusters>("inliers", "Point Cloud clusters.");
    outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_models", "Bounded plane coefficients: (x, y, z, normal_x, normal_y, normal_z, principal_axis_x, principal_axis_y, principal_axis_z, width");
    outputs.declare<std::vector<UnalignedVector4f> >("centroids", "Plane centroids.");
    outputs.declare<std::vector<UnalignedVector3f> >("sizes", "Plane sizes.");
    outputs.declare<std::vector<UnalignedAffine3f> >("transforms", "Plane transforms.");

    outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("polygons", "Fitted 3D Polygons.");

    outputs.declare<UnalignedVector4f>("centroid_biggest", "The centroid of the biggest plane found.");
    outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("models_biggest", "Plane coefficients");
    outputs.declare<UnalignedAffine3f>("transform_biggest", "The homogenous transformation of the biggest plane found.");
    outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("bounded_model_biggest", "Plane coefficients of the biggest plane");
    outputs.declare<std::vector< ::pcl::ModelCoefficientsConstPtr> >("polygon_biggest", "Plane coefficients");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    clusters_ = inputs["clusters"];
    plane_coefficients_ = outputs["models"];
    bounded_plane_coefficients_ = outputs["bounded_models"];
    plane_centroids_ = outputs["centroids"];
    plane_sizes_ = outputs["sizes"];
    plane_transforms_ = outputs["transforms"];
    plane_coefficients_biggest_ = outputs["models_biggest"];
    plane_centroid_biggest_ = outputs["centroid_biggest"];
    plane_transform_biggest_ = outputs["transform_biggest"];
    bounded_plane_coefficients_biggest_= outputs["bounded_model_biggest"];
    polygon_biggest_ = outputs["polygon_biggest"];
    fit_quality_ = outputs["fit_quality"];
    inliers_ = outputs["inliers"];
    polygons_ = outputs["polygons"];

    min_inlier_ratio_ = params["min_inlier_ratio"];
    distance_threshold_ = params["distance_threshold"];
    min_boxness_ = params["min_boxness"];
    weight_contour_ = params["weight_contour"];
    max_size_ = params["max_size"];
    min_size_ = params["min_size"];
    polygon_approximation_epsilon_ = params["polygon_approximation_epsilon"];
    
    publish_rviz_markers_ = params["publish_rviz_markers"];
  }

  void publishRVizMarkers(const std::string& frame_id)
  {
    static ros::NodeHandle nh;
    static ros::Publisher marker_publisher = nh.advertise<visualization_msgs::Marker>("/box_fits", 20);
    static ros::Publisher obstacle_publisher = nh.advertise<visualization_msgs::Marker>("/obstacle_fits", 10);

    // publish biggest plane
    if (!bounded_plane_coefficients_->empty())
    {
      int i = biggest_index;

      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time();
      marker.lifetime = ros::Duration(0.2);
      marker.id = 0;
      marker.ns = "boxes";
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      Eigen::Vector3f normal(bounded_plane_coefficients_->at(i)->values[3], bounded_plane_coefficients_->at(i)->values[4], bounded_plane_coefficients_->at(i)->values[5]);
      Eigen::Vector3f principal_axis(bounded_plane_coefficients_->at(i)->values[6], bounded_plane_coefficients_->at(i)->values[7], bounded_plane_coefficients_->at(i)->values[8]);
      float height = principal_axis.norm();
      float width = bounded_plane_coefficients_->at(i)->values[9];
      principal_axis.normalize();
      normal.normalize();

//      Eigen::Affine3f orientation = ::pcl::getTransformationFromTwoUnitVectors(normal.cross(yaw), normal);
      Eigen::Matrix3f orientation;
      orientation << principal_axis, normal.cross(principal_axis), normal;
      Eigen::Quaternionf q(orientation);

//      Eigen::Vector3f cross_prod = Eigen::Vector3f::UnitZ().cross(normal);
//      float dot_prod = Eigen::Vector3f::UnitZ().dot(normal);
//      Eigen::AngleAxisf aa(acos(dot_prod), cross_prod);
//      Eigen::Quaternionf q(aa);

      marker.pose.position.x = bounded_plane_coefficients_->at(i)->values[0];
      marker.pose.position.y = bounded_plane_coefficients_->at(i)->values[1];
      marker.pose.position.z = bounded_plane_coefficients_->at(i)->values[2];

      tf::Quaternion tf_q;
      tf::quaternionEigenToTF(q.cast<double>(), tf_q);
      tf::quaternionTFToMsg(tf_q, marker.pose.orientation);

      marker.scale.x = height;
      marker.scale.y = width;
      marker.scale.z = *distance_threshold_;
      marker.color.a = 0.5;
      marker.color.b = 1.0;
      marker.color.g = marker.color.r = 0.0;

      obstacle_publisher.publish(marker);

      // publish as a transform
      static tf::TransformBroadcaster tf_broadcaster;
       geometry_msgs::TransformStamped msg;
       msg.transform.rotation = marker.pose.orientation;
       msg.transform.translation.x = marker.pose.position.x;
       msg.transform.translation.y = marker.pose.position.y;
       msg.transform.translation.z = marker.pose.position.z;
       msg.header.stamp = ros::Time::now();
       msg.header.frame_id = frame_id;
       msg.child_frame_id = "biggest";// + boost::lexical_cast<std::string>(i);
       tf_broadcaster.sendTransform(msg);
    }

    // publish all planes as marker messages
    for (size_t i = 0; i < bounded_plane_coefficients_->size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time();
      marker.lifetime = ros::Duration(0.2);
      marker.id = i;
      marker.ns = "boxes";
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      Eigen::Vector3f normal(bounded_plane_coefficients_->at(i)->values[3], bounded_plane_coefficients_->at(i)->values[4], bounded_plane_coefficients_->at(i)->values[5]);
      Eigen::Vector3f principal_axis(bounded_plane_coefficients_->at(i)->values[6], bounded_plane_coefficients_->at(i)->values[7], bounded_plane_coefficients_->at(i)->values[8]);
      float height = principal_axis.norm();
      float width = bounded_plane_coefficients_->at(i)->values[9];
      principal_axis.normalize();
      normal.normalize();

//      Eigen::Affine3f orientation = ::pcl::getTransformationFromTwoUnitVectors(normal.cross(yaw), normal);
      Eigen::Matrix3f orientation;
      orientation << principal_axis, normal.cross(principal_axis), normal;
      Eigen::Quaternionf q(orientation);
      Eigen::Affine3f affine;

//      Eigen::Vector3f cross_prod = Eigen::Vector3f::UnitZ().cross(normal);
//      float dot_prod = Eigen::Vector3f::UnitZ().dot(normal);
//      Eigen::AngleAxisf aa(acos(dot_prod), cross_prod);
//      Eigen::Quaternionf q(aa);

      marker.pose.position.x = bounded_plane_coefficients_->at(i)->values[0];
      marker.pose.position.y = bounded_plane_coefficients_->at(i)->values[1];
      marker.pose.position.z = bounded_plane_coefficients_->at(i)->values[2];

      tf::Quaternion tf_q;
      tf::quaternionEigenToTF(q.cast<double>(), tf_q);
      tf::quaternionTFToMsg(tf_q, marker.pose.orientation);

      marker.scale.x = height;
      marker.scale.y = width;
      marker.scale.z = *distance_threshold_;
      marker.color.a = 0.5;
      marker.color.b = 1.0;
      marker.color.g = marker.color.r = 0.0;

      marker_publisher.publish(marker);
    }
  }

  template<typename Point>
    void calculateBoundedPlane(boost::shared_ptr<const ::pcl::PointCloud<Point> >& cloud,
                                  const std::vector<int>& indices,
                                  Eigen::Vector3f& center, const Eigen::Vector3f& normal,
                                  double& rectangularity, Eigen::Vector3f& principal_axis,
                                  float& width, float& height, bool first)
    {
      std::vector< ::cv::Point2i> points_2d, points_convex_hull;

      // project the x-axis of the camera's optical frame into the plane
      Eigen::Vector3f x_axis = Eigen::Vector3f::UnitX() - Eigen::Vector3f::UnitX().dot(normal) * normal;
      x_axis.normalize();

      Eigen::Affine3f transformToPlaneCoordinates;
      ::pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal.cross(x_axis), normal, center, transformToPlaneCoordinates);

      for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); ++it) {
        Eigen::Vector3f uv = transformToPlaneCoordinates * (*cloud)[*it].getVector3fMap();
        ::cv::Point2i pt(uv[0] * 1000, uv[1] * 1000);
        points_2d.push_back(pt);
      }

      cv::Mat dbg(points_2d);
      ::cv::RotatedRect rectangular_boundary = ::cv::minAreaRect(points_2d);
      ::cv::convexHull(points_2d, points_convex_hull, true, true);

      // for debugging
      /*static ::cv::Mat image(600, 800, CV_8UC1);
      static int coutner = 0;
      if (first) {
        image.setTo(0);
        coutner = 0;
      }
      ::cv::Point2i centre(100 + 100 * (coutner % 4), 100 + (coutner / 4) * 100);
      for (size_t i = 0; i < points_2d.size(); ++i) {
        ::cv::circle(image, centre + points_2d[i], 1, 255);
      }
      rectangular_boundary.center += cv::Point2f(1.0 * centre.x, 1.0 * centre.y);
      ::cv::ellipse(image, rectangular_boundary, 122);
      ::cv::imshow("test", image);
      ::cv::waitKey(10);
      coutner++;*/

      std::vector< ::cv::Point2i> approx_hull;
      ::cv::approxPolyDP(points_convex_hull, approx_hull, *polygon_approximation_epsilon_, true); // 5cm precision


      ModelCoefficients::Ptr polygon(new ModelCoefficients());
      polygon->values.resize(approx_hull.size() * 3);

//      ::geometry_msgs::PolygonStamped poly;
//      poly.header = pcl_conversions::fromPCL(cloud->header);

      for (size_t i = 0; i < approx_hull.size(); ++i) {
          Eigen::Vector3f plane_coordinates(approx_hull[i].x * 0.001, approx_hull[i].y * 0.001, 0);
          Eigen::Vector3f cartesian_coordinates = transformToPlaneCoordinates.inverse() * plane_coordinates;

          polygon->values[3 * i] = cartesian_coordinates(0);
          polygon->values[3 * i + 1] = cartesian_coordinates(1);
          polygon->values[3 * i + 2] = cartesian_coordinates(2);

          // That's for visualization.
          /*
          ::geometry_msgs::Point32 p;
          p.x = cartesian_coordinates(0);
          p.y = cartesian_coordinates(1);
          p.z = cartesian_coordinates(2);
          poly.polygon.points.push_back(p);
          */
      }
      polygons_->push_back(polygon);

      /*
      static ::ros::NodeHandle nh;
      static ::ros::Publisher pub = nh.advertise< ::geometry_msgs::PolygonStamped>("somepoly_1", 10);
      static ::ros::Publisher pub2 = nh.advertise< ::geometry_msgs::PolygonStamped>("somepoly_2", 10);
      static ::ros::Publisher pub3 = nh.advertise< ::geometry_msgs::PolygonStamped>("somepoly_3", 10);
      static ::ros::Publisher pub4 = nh.advertise< ::geometry_msgs::PolygonStamped>("somepoly_4", 10);
      static ::ros::Publisher pub5 = nh.advertise< ::geometry_msgs::PolygonStamped>("somepoly_5", 10);
      static int pub_count = 0;

      switch (pub_count)
      {
      case 0:
          pub.publish(poly);
          break;
      case 1:
          pub2.publish(poly);
          break;
      case 2:
          pub3.publish(poly);
          break;
      case 3:
          pub4.publish(poly);
          break;
      case 4:
          pub5.publish(poly);
          break;
      }
      pub_count = (pub_count + 1) % 5;
      */
      double area_convex_hull = ::cv::contourArea(points_convex_hull);

      rectangularity = area_convex_hull / rectangular_boundary.size.area();

      // make the measure a bit more harsh -- circle is minimum
      rectangularity = std::max(1.0 - ((1.0 - rectangularity) / (1.0 - M_PI / 4.0)), 0.0);

//      ROS_INFO("Plane with %zu points and %f area makes: %f", indices.size(), rectangular_boundary.size.area(), rectangularity);

      principal_axis = Eigen::AngleAxisf(::pcl::deg2rad(rectangular_boundary.angle), normal) * x_axis;
//      Eigen::Quaternionf q(Eigen::AngleAxisf(::pcl::deg2rad(rectangular_boundary.angle), normal));
//
//      principal_axis = transformToPlaneCoordinates.inverse() * q._transformVector(Eigen::Vector3f::UnitX());
//      principal_axis.normalize();

      Eigen::Vector3f new_center(rectangular_boundary.center.x * 0.001, rectangular_boundary.center.y * 0.001, 0);
      center = transformToPlaneCoordinates.inverse() * new_center;

      height = rectangular_boundary.size.width * 0.001;
      principal_axis *= height;
      width = rectangular_boundary.size.height * 0.001;
    }

  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input,
                boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
    {
      biggest_index = 0;
      plane_coefficients_->clear();
      bounded_plane_coefficients_->clear();
      bounded_plane_coefficients_biggest_->clear();
      plane_centroids_->clear();
      plane_transforms_->clear();
      plane_sizes_->clear();
      plane_coefficients_biggest_->clear();
      polygons_->clear();
      polygon_biggest_->clear();
      fit_quality_->clear();
      inliers_->clear();

      // for debugging
      ::pcl::PointCloud< ::pcl::PointXYZRGB>::Ptr box_fit_error(new ::pcl::PointCloud< ::pcl::PointXYZRGB>);
      ::pcl::copyPointCloud(*input, *box_fit_error);

      int biggest_size = 0;

      // iterate through segmented clusters
//      for (std::vector<std::vector<int> >::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
      for (ecto::pcl::Clusters::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
      {
        ::pcl::SACSegmentationFromNormals<Point, ::pcl::Normal> sac_seg;
//          ::pcl::SACSegmentation<Point> sac_seg;
        Indices::Ptr inliers(new Indices());
        ModelCoefficients::Ptr model(new ModelCoefficients());

        sac_seg.setModelType(::pcl::SACMODEL_NORMAL_PLANE);
        sac_seg.setNormalDistanceWeight(0.1);
        sac_seg.setMethodType(::pcl::SAC_RANSAC);
        sac_seg.setEpsAngle(::pcl::deg2rad(20.0));//sac_seg.setEpsAngle(*eps_angle_);
        sac_seg.setDistanceThreshold(*distance_threshold_);
        //          sac_seg.setMaxIterations(*max_iterations_);
//            sac_seg.setOptimizeCoefficients(true);
        //          sac_seg.setProbability(*probability_);
        //          sac_seg.setAxis(Eigen::Vector3f(*axis_x_, *axis_y_, *axis_z_));
        sac_seg.setInputCloud(input);
        sac_seg.setInputNormals(normals);
        sac_seg.setIndices(boost::make_shared<std::vector<int> >(it->indices));
        sac_seg.segment(*inliers, *model);

        if (inliers->indices.empty())
          continue;

        double inlier_ratio = static_cast<double>(inliers->indices.size()) / it->indices.size();

        Eigen::Vector4f centroid_4d;
        ::pcl::compute3DCentroid(*input, inliers->indices, centroid_4d);

        plane_centroids_->push_back(centroid_4d);

        double rectangularity;
        float width, height;
        Eigen::Vector3f normal(model->values[0], model->values[1], model->values[2]);
        Eigen::Vector3f centroid = centroid_4d.topRows(3);

        // switch normal if it is not showing towards the viewpoint of the sensor
        if (normal.dot(centroid) < 0) {
          normal = -normal;
          model->values[3] = -model->values[4];
        }

        model->values[0] = normal(0);
        model->values[1] = normal(1);
        model->values[2] = normal(2);
        plane_coefficients_->push_back(model);

        normal.normalize();

        Eigen::Vector3f principal_axis;

        // Attention: before this was only computed about the inliers! inliers->indices
        calculateBoundedPlane(input, it->indices, centroid, normal, rectangularity, principal_axis, width, height, it == clusters_->begin());

        // do the weighting!
//            double t = (1.0 - *weight_contour_) / *weight_contour_;
//            double boxness_weighted = (t + 1.0 / t - 2.0) * boxness * boxness * boxness + (-2.0 * t - (1.0 / t) + 3.0) * boxness * boxness + t * boxness;
//            boxness_weighted = std::max(std::min(boxness_weighted, 1.0), 0.0);
//
//            t = (1.0 - *weight_flatness_) / *weight_flatness_;
//            double flatness_weighted = (t + 1.0 / t - 2.0) * flatness * flatness * flatness + (-2.0 * t - (1.0 / t) + 3.0) * flatness * flatness + t * flatness;
//            flatness_weighted = std::max(std::min(flatness_weighted, 1.0), 0.0);
//
//            ROS_INFO("flattness (weighted) vs boxness (weighted): %f (%f) vs %f (%f)", boxness, boxness_weighted, flatness, flatness_weighted);
//
//            double boxness = boxness_weighted * flatness_weighted;

        double boxness = (1.0 - *weight_contour_) * inlier_ratio + *weight_contour_ * rectangularity;

        // publish inliers to debug
        for (std::vector<int>::iterator jt = it->indices.begin(); jt != it->indices.end(); ++jt)
        {
          ::pcl::PointXYZRGB& p = box_fit_error->at(*jt);
          p.r = boxness * 255.0;
          p.g = 0;
          p.b = (1.0 - boxness) * 255.0;
        }

        if (inlier_ratio < *min_inlier_ratio_ || boxness < *min_boxness_)
          continue;

//            ROS_INFO("new boxness (incl. %zu / %zu):   %f", inliers->indices.size(), it->size(), boxness);
//          ROS_INFO("%.2f rectangularity (%.3f) + %.2f flattness (%.3f) = %.3f", 1-*weight_flatness_, rectangularity, *weight_flatness_, flatness, boxness);

//        if (width < *max_size_ && height < *max_size_) {
          ModelCoefficients::Ptr bounded_model(new ModelCoefficients());
          bounded_model->values.resize(10);
          bounded_model->values[0] = centroid[0];
          bounded_model->values[1] = centroid[1];
          bounded_model->values[2] = centroid[2];
          bounded_model->values[3] = normal[0];
          bounded_model->values[4] = normal[1];
          bounded_model->values[5] = normal[2];
          bounded_model->values[6] = principal_axis[0];
          bounded_model->values[7] = principal_axis[1];
          bounded_model->values[8] = principal_axis[2];
          bounded_model->values[9] = width;

          bounded_plane_coefficients_->push_back(bounded_model);
          fit_quality_->push_back(boxness);
          inliers_->push_back(*inliers);
//        }

          Eigen::Matrix3f rotation;
          principal_axis.normalize();
          rotation << principal_axis, normal.cross(principal_axis), normal;
          UnalignedAffine3f transform = Eigen::Translation3f(centroid[0], centroid[1], centroid[2]) * rotation;
          plane_transforms_->push_back(transform);

          UnalignedVector3f size(height, width, 0);
          plane_sizes_->push_back(size);

          if (inliers->indices.size() > biggest_size) {
            if (plane_coefficients_biggest_->empty()) {
              plane_coefficients_biggest_->push_back(model);
              bounded_plane_coefficients_biggest_->push_back(bounded_model);
              polygon_biggest_->push_back(polygons_->back());
            }
            else {
              plane_coefficients_biggest_->at(0) = model;
              bounded_plane_coefficients_biggest_->at(0) = bounded_model;
              polygon_biggest_->at(0) = polygons_->back();
            }
            biggest_index = bounded_plane_coefficients_->size() - 1;
            biggest_size = inliers->indices.size();
            
            (*plane_centroid_biggest_) = centroid_4d;
            (*plane_transform_biggest_) = transform;// * Eigen::AngleAxisf(a, axis);
            //ROS_INFO("Outgoing transform: %f %f %f", plane_transform_biggest_->translation()[0], plane_transform_biggest_->translation()[1], plane_transform_biggest_->translation()[2]);
          }
      }

      ROS_INFO("Found %zu planes and %zu big ones.", plane_coefficients_->size(), plane_coefficients_biggest_->size());

      if (*publish_rviz_markers_)
      {
          static ros::NodeHandle nh;
          static ros::Publisher fit_error = nh.advertise<sensor_msgs::PointCloud2>("/box_fit_error", 1);

          sensor_msgs::PointCloud2Ptr color_msg(new sensor_msgs::PointCloud2);
          ::pcl::toROSMsg(*box_fit_error, *color_msg);
          color_msg->header = pcl_conversions::fromPCL(input->header);
          fit_error.publish(*color_msg);
          
          publishRVizMarkers(input->header.frame_id);
      }

      return OK;
    }
};


struct PlaneFits2D
{
  spore<ecto::pcl::Clusters> clusters_;
  spore<UnalignedVector3f> normal_;
  spore<UnalignedVector4f> height_origin_;
  
  spore<std::vector<UnalignedVector3f> > bbox_sizes_;
  spore<std::vector<UnalignedAffine3f> > bbox_transforms_;
  
  spore<double> min_aspect_ratio_;
  spore<double> max_aspect_ratio_;

  spore<bool> publish_rviz_markers_;
  
  static void declare_params(ecto::tendrils& params)
  {
    params.declare<double>("min_aspect_ratio", "Filters out all boxes with minimum aspect ratio (= longer side / shorter side; between 1 and inf).", 1.0);
    params.declare<double>("max_aspect_ratio", "Filters out all boxes with maximum aspect ratio (= longer side / shorter side; between 1 and inf).", FLT_MAX);
    params.declare<bool>("publish_rviz_markers", "Should the output be published for visualization?", false);
  }

  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
  {
    inputs.declare< ecto::pcl::Clusters >("clusters", "Point Cloud clusters.");
    inputs.declare< UnalignedVector3f >("normal", "Normal vector of all 2d bounding box footprints.").required(true);
    inputs.declare< UnalignedVector4f >("height_origin", "If this is supplied, then calculate height of bounding box based on the largest distance of any point to this point along the normal").required(false);

    outputs.declare<std::vector<UnalignedVector3f> >("sizes", "Bounding box sizes.");
    outputs.declare<std::vector<UnalignedAffine3f> >("transforms", "Bounding box transforms.");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    clusters_ = inputs["clusters"];
    normal_ = inputs["normal"];
    height_origin_ = inputs["height_origin"];
    
    bbox_sizes_ = outputs["sizes"];
    bbox_transforms_ = outputs["transforms"];
    
    min_aspect_ratio_ = params["min_aspect_ratio"];
    max_aspect_ratio_ = params["max_aspect_ratio"];
    
    publish_rviz_markers_ = params["publish_rviz_markers"];
  }
  
  void publishRVizMarkers(const std::vector<UnalignedAffine3f>& transforms, const std::vector<UnalignedVector3f>& sizes)
  {
      static ros::NodeHandle nh("~");
      static ros::Publisher marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("/bounding_boxes", 10);
      visualization_msgs::MarkerArray markers;
      for (size_t i = 0; i < transforms.size(); ++i)
      {
          visualization_msgs::Marker marker;
          marker.header.frame_id = "/camera_rgb_optical_frame";
          marker.header.stamp = ros::Time();
          //marker.lifetime = ros::Duration(0);
          marker.id = i;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          
          tf::Pose bbox_pose;
          tf::poseEigenToTF((Eigen::Affine3d) (transforms[i].cast<double>()), bbox_pose);
          tf::poseTFToMsg(bbox_pose, marker.pose);
          
          marker.scale.x = std::max(sizes[i][0], 0.01f);
          marker.scale.y = std::max(sizes[i][1], 0.01f);
          marker.scale.z = std::max(sizes[i][2], 0.01f);
          marker.color.a = 0.5;
          marker.color.b = 1.0;
          marker.color.g = marker.color.r = 0.0;
          
          markers.markers.push_back(marker);
      }
      marker_publisher.publish(markers);
  }
  
  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
      bbox_transforms_->clear();
      bbox_sizes_->clear();
      
      // for each cluster calculate the 2d box
      for (ecto::pcl::Clusters::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
      {
          //calculateBoundedPlane(input, it->indices, centroid, normal, principal_axis, width, height, it == clusters_->begin());
          ::Eigen::Vector4f centroid_4d;
          ::pcl::compute3DCentroid(*input, it->indices, centroid_4d);
          
          // project the x-axis of the camera's optical frame into the plane
          Eigen::Vector3f x_axis = Eigen::Vector3f::UnitX() - Eigen::Vector3f::UnitX().dot(*normal_) * (*normal_);
          x_axis.normalize();
          
          Eigen::Affine3f transformToPlaneCoordinates;
          ::pcl::getTransformationFromTwoUnitVectorsAndOrigin(normal_->cross(x_axis), *normal_, centroid_4d.head<3>(), transformToPlaneCoordinates);
          
          std::vector< ::cv::Point2i> points_2d;
          double max_along_normal = -10e19;
          double min_along_normal = 10e19;
          for (std::vector<int>::const_iterator jt = it->indices.begin(); jt != it->indices.end(); ++jt)
          {
            ::Eigen::Vector3f uv = transformToPlaneCoordinates * (*input)[*jt].getVector3fMap();
            ::cv::Point2i pt(uv[0] * 1000, uv[1] * 1000);
            points_2d.push_back(pt);
            
            double dot_prod = (*input)[*jt].getVector3fMap().dot(*normal_);
            if (dot_prod > max_along_normal)
                max_along_normal = dot_prod;
            if (dot_prod < min_along_normal)
                min_along_normal = dot_prod;
          }
          ::cv::RotatedRect rectangular_boundary = ::cv::minAreaRect(points_2d);
          
          ROS_INFO_STREAM("angle: " << rectangular_boundary.angle << "  and size: " << rectangular_boundary.size.width << " " << rectangular_boundary.size.height);
          
          Eigen::Vector3f principal_axis = Eigen::AngleAxisf(::pcl::deg2rad(rectangular_boundary.angle), *normal_) * x_axis;
          Eigen::Vector3f new_center(rectangular_boundary.center.x * 0.001, rectangular_boundary.center.y * 0.001, 0);
          new_center = transformToPlaneCoordinates.inverse() * new_center;
          
          double height = rectangular_boundary.size.height * 0.001;
          double width = rectangular_boundary.size.width * 0.001;
          double aspect_ratio = std::max(width, height) / std::min(width, height);
          //std::cout << "Aspect Ratio: " << aspect_ratio << std::endl;
          if (aspect_ratio < *min_aspect_ratio_ || aspect_ratio > *max_aspect_ratio_)
            continue;
          
          Eigen::Matrix3f rotation;

 	  //Eigen::Vector3f negate_normal = (-1.0)*(*normal_);

          if (width > height)
          {
            rotation << principal_axis, normal_->cross(principal_axis), *normal_;
	    //rotation << normal_->cross(principal_axis),principal_axis,  negate_normal;	
          }
          else
          {
            rotation << principal_axis.cross(*normal_), principal_axis, *normal_;
	    //rotation << principal_axis, principal_axis.cross(*normal_),  negate_normal;
            ::std::swap(width, height);
          }
          
          double depth;
          if (height_origin_.user_supplied())
          {
              depth = (height_origin_->head<3>().dot(*normal_)) - min_along_normal;
              
              new_center = new_center - (*normal_)*(new_center.dot(*normal_)-(height_origin_->head<3>().dot(*normal_))) - (*normal_ * (depth * 0.5));
          }
          else
          {
              depth = (max_along_normal - min_along_normal);
          }
          
          UnalignedAffine3f transform = Eigen::Translation3f(new_center[0], new_center[1], new_center[2]) * rotation;
          bbox_transforms_->push_back(transform);
          
          UnalignedVector3f size(width, height, depth);
          bbox_sizes_->push_back(size);
      }
      
      
      if (*publish_rviz_markers_)
          publishRVizMarkers(*bbox_transforms_, *bbox_sizes_);
      
      return OK;
    }
};
}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCellWithNormals<ecto_rbo_pcl::PlaneFits>, "PlaneFits", "Plane Segmentation using Sample Consensus.");
ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::PlaneFits2D>, "PlaneFits2D", "Plane Fitting using smallest enclosing rectangle.");
