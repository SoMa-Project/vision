/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

/*
 * OrientationSet.h
 *
 *  Created on: Dec 21, 2010
 *      Author: clemens
 */

#ifndef ORIENTATIONSET_H_
#define ORIENTATIONSET_H_

#include <vector>
#include <bitset>

#include <tf/tf.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

namespace posesets
{

typedef ::Eigen::Matrix<float, 3, 3, ::Eigen::DontAlign> RotationMatrix;
//typedef ::Eigen::Vector3f Coordinates;
typedef ::Eigen::Quaternionf Coordinates;
//typedef ::Eigen::Vector3i CoordinateIds;
typedef int CoordinateIds;

class OrientationSet {
public:
    OrientationSet();
    virtual ~OrientationSet();

    int getViewId(const Coordinates& view);

    std::vector<Coordinates>& getOrientations();
    const std::vector<CoordinateIds>& getSet() const;
    void setSet(const std::vector<CoordinateIds>& new_set);

    void add(const tf::Quaternion& orientation);
    void addFuzzy(const tf::Quaternion& orientation);
    void add(const CoordinateIds& orientation);
    void add(const RotationMatrix& orientation);
    void add(const tf::Quaternion& orientation, const tf::Vector3& around_axis);
    void addFuzzy(const tf::Quaternion& orientation, const tf::Vector3& around_axis);
    void add(const RotationMatrix& orientation, const tf::Vector3& around_axis);

    void addAll();
    void addHalf();

    bool hasIntersection(const OrientationSet& other_set) const;
    void intersect(const OrientationSet& other_set);
    OrientationSet getIntersection(const OrientationSet& other_set) const;

    RotationMatrix sample() const;

    bool empty();

private:
    inline void init() { init4608(); }
    void initVector(double arr[], int size);

    // new stuff
    void init72();
    void init576();
    void init4608();

    std::vector<Coordinates> orientations;
    std::vector<CoordinateIds> set;


    int dbg;
};

typedef boost::shared_ptr< ::posesets::OrientationSet > OrientationSetPtr;
typedef boost::shared_ptr< ::posesets::OrientationSet const> OrientationSetConstPtr;

}

#endif /* ORIENTATIONSET_H_ */
