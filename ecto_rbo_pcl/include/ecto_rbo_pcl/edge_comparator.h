/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

#ifndef ECTO_RBO_PCL_SEGMENTATION_EDGE_COMPARATOR_H_
#define ECTO_RBO_PCL_SEGMENTATION_EDGE_COMPARATOR_H_

#include <pcl/common/angles.h>
//#include "comparator.h"
#include <boost/make_shared.hpp>

namespace ecto_rbo_pcl
{
  /** \brief EdgeComparator is a Comparator that operates on plane coefficients, for use in planar segmentation.
    * In conjunction with OrganizedConnectedComponentSegmentation, this allows planes to be segmented from organized data.
    *
    */
template<typename PointT, typename PointNT>
  class EdgeComparator : public pcl::Comparator<PointT>
  {
  public:
    typedef typename pcl::Comparator<PointT>::PointCloud PointCloud;
    typedef typename pcl::Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;

    typedef typename pcl::PointCloud<PointNT> PointCloudN;
    typedef typename PointCloudN::Ptr PointCloudNPtr;
    typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

    typedef boost::shared_ptr<EdgeComparator<PointT, PointNT> > Ptr;
    typedef boost::shared_ptr<const EdgeComparator<PointT, PointNT> > ConstPtr;

    using pcl::Comparator<PointT>::input_;

    /** \brief Empty constructor for EdgeComparator. */
    EdgeComparator() :
        normals_(), distance_threshold_(0.02f), curvature_threshold_(0.1), curvature_dist_threshold_(0.01)
    {
    }

    /** \brief Destructor for EdgeComparator. */
    virtual ~EdgeComparator()
    {
    }

    virtual void setInputCloud(const PointCloudConstPtr& cloud)
    {
      input_ = cloud;
    }

    /** \brief Provide a pointer to the input normals.
     * \param[in] normals the input normal cloud
     */
    inline void setInputNormals(const PointCloudNConstPtr &normals)
    {
      normals_ = normals;
    }

    /** \brief Get the input normals. */
    inline PointCloudNConstPtr getInputNormals() const
    {
      return (normals_);
    }

    /** \brief Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
     * \param[in] distance_threshold the tolerance in meters (at 1m)
     * \param[in] depth_dependent whether to scale the threshold based on range from the sensor (default: false)
     */
    void setDistanceThreshold(float distance_threshold, bool depth_dependent = false)
    {
      distance_threshold_ = distance_threshold;
    }

    void setCurvatureThreshold(float curvature_threshold)
    {
      curvature_threshold_ = curvature_threshold;
    }

    void setCurvatureDistanceThreshold(float curvature_dist_threshold)
    {
      curvature_dist_threshold_ = curvature_dist_threshold;
    }

    /** \brief Get the distance threshold in meters (d component of plane equation) between neighboring points, to be considered part of the same plane. */
    inline float getDistanceThreshold() const
    {
      return (distance_threshold_);
    }

    inline float getCurvatureThreshold() const
    {
      return (curvature_threshold_);
    }

    inline float getCurvatureDistanceThreshold() const
    {
      return (curvature_dist_threshold_);
    }

    /** \brief Compare points at two indices by their plane equations.  True if the angle between the normals is less than the angular threshold,
     * and the difference between the d component of the normals is less than distance threshold, else false
     * \param idx1 The first index for the comparison
     * \param idx2 The second index for the comparison
     */
    virtual bool compare(int idx1, int idx2) const
    {
      // check if curvature high
      bool curvature_high_enough = fabs(normals_->points[idx1].curvature) > curvature_threshold_;

      // and similar (can curvature be negative and positive --> convex vs concave)
      bool curvature_similar_enough = fabs(normals_->points[idx1].curvature - normals_->points[idx2].curvature) < curvature_dist_threshold_;

      // and check if not far apart
      float dx = input_->points[idx1].x - input_->points[idx2].x;
      float dy = input_->points[idx1].y - input_->points[idx2].y;
      float dz = input_->points[idx1].z - input_->points[idx2].z;
      float dist = sqrtf(dx * dx + dy * dy + dz * dz);

      bool distance_small_enough = dist < distance_threshold_;

      return curvature_high_enough && curvature_similar_enough && distance_small_enough;
    }

    protected:
      PointCloudNConstPtr normals_;
      float distance_threshold_;
      float curvature_threshold_;
      float curvature_dist_threshold_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif // ECTO_RBO_PCL_SEGMENTATION_EDGE_COMPARATOR_H_
