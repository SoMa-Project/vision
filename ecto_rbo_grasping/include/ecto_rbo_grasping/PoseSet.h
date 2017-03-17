/*
 * PoseSet.h
 *
 *  Created on: Dec 21, 2010
 *      Author: clemens
 */

#ifndef POSESET_H_
#define POSESET_H_

#include <vector>
#include <tf/tf.h>
#include <boost/shared_ptr.hpp>
#include <ecto_rbo_grasping/OrientationSet.h>

#include "Wm5Vector3.h"
#include "Wm5Segment3.h"
#include "Wm5Box3.h"

namespace posesets
{

class PoseSet {
public:
    PoseSet(const tf::Transform& origin);
    virtual ~PoseSet();

    const tf::Transform& getOrigin() const;

    OrientationSet& getOrientations();
    const OrientationSet& getOrientations() const;

    const tf::Vector3& getPositions() const;

    void setOrigin(const tf::Transform& origin);
    void setOrientations(const OrientationSet& orientations);
    void setPositions(const tf::Vector3& positions);

    bool isIntersecting(const PoseSet& other) const;

    bool intersect(const PoseSet& other);
    bool intersectBoxes(const tf::Transform& a, const tf::Vector3& extents_a, const tf::Transform& b, const tf::Vector3& extents_b);
    void intersectBoxes(const tf::Transform& a, const tf::Vector3& extents_a, const tf::Transform& b, const tf::Vector3& extents_b, tf::Transform& n, tf::Vector3& extents_n);
    void intersectBoxesCGAL(const tf::Transform& a, const tf::Vector3& extents_a, const tf::Transform& b, const tf::Vector3& extents_b, tf::Transform& n, tf::Vector3& extents_n);
    bool intersectBoxesGeometricTools(const tf::Transform& a, const tf::Vector3& extents_a, const tf::Transform& b, const tf::Vector3& extents_b, tf::Transform& n, tf::Vector3& extents_n);

    std::vector<Wm5::Vector3d> intersection_points;

    void updateBoxVertices(Wm5::Vector3d box_vertices[8], Wm5::Segment3d edges[12]) const;
    Wm5::Box3d getWm5Box3d() const;

private:
    tf::Transform origin;
    OrientationSet orientations;
    tf::Vector3 positions;
};

typedef std::vector< ::posesets::PoseSet > PoseSetArray;
typedef boost::shared_ptr< ::posesets::PoseSetArray > PoseSetArrayPtr;
typedef boost::shared_ptr< ::posesets::PoseSetArray const > PoseSetArrayConstPtr;

typedef boost::shared_ptr< ::posesets::PoseSet > PoseSetPtr;
typedef boost::shared_ptr< ::posesets::PoseSet const> PoseSetConstPtr;

}

#endif /* POSESET_H_ */
