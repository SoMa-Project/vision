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
//    void init89();
//    void init136();
//    void init362();

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
