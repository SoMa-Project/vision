#ifndef ENVIRONMENTALCONSTRAINTREGION_H
#define ENVIRONMENTALCONSTRAINTREGION_H

#include <tf/transform_datatypes.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Spherical_kernel_3.h>
#include <CGAL/Exact_spherical_kernel_3.h>
#include <CGAL/Algebraic_kernel_for_spheres_2_3.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Circular_arc_3.h>
#include <CGAL/Circular_arc_point_3.h>
#include <CGAL/Circle_3.h>
#include <CGAL/Vector_3.h>

typedef double NT;
typedef CGAL::Cartesian<NT> K;
typedef CGAL::Algebraic_kernel_for_spheres_2_3<K> Algebraic_K;
//typedef CGAL::Spherical_kernel_3<K, Algebraic_K> Spherical_K;
typedef CGAL::Exact_spherical_kernel_3 Spherical_K;
typedef CGAL::Segment_3<K> Segment_3;
typedef CGAL::Plane_3<K> Plane_3;
typedef CGAL::Point_3<Spherical_K> Point_3;
typedef CGAL::Vector_3<Spherical_K> Vector_3;
typedef CGAL::Circle_3<Spherical_K> Circle_3;
typedef CGAL::Circular_arc_3<Spherical_K> Circular_arc_3;
typedef CGAL::Circular_arc_point_3<Spherical_K> Circular_arc_point_3;

class EnvironmentalConstraintRegion
{
public:
    EnvironmentalConstraintRegion();
    EnvironmentalConstraintRegion(const tf::Transform& origin, const tf::Vector3& translation_boundaries, const tf::Vector3& orientation_boundaries);

    void setOrigin(const tf::Transform& origin);
    void setTranslationBoundaries(const tf::Vector3& translation_boundaries);
    void setOrientationBoundaries(const tf::Vector3& orientation_boundaries);

    const tf::Transform& getOrigin() const;
    const tf::Matrix3x3& getOrientationOrigin() const;
    const tf::Vector3& getOrientationBoundaries1() const;
    const tf::Vector3& getOrientationBoundaries2() const;
    const Plane_3 &getPlane() const;
    bool intersect(const EnvironmentalConstraintRegion& other, EnvironmentalConstraintRegion& result);

private:
    void createAxisRegionOnSphere(const tf::Matrix3x3& axes, int axis, float width, float height, std::vector<Circular_arc_3> edges);

    tf::Transform origin;
    Plane_3 plane;

    tf::Vector3 translation_boundaries;

    tf::Matrix3x3 orientation_origin;
    tf::Vector3 orientation_boundaries_1;
    tf::Vector3 orientation_boundaries_2;
};

#endif // ENVIRONMENTALCONSTRAINTREGION_H
