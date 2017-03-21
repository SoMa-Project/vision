/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

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
