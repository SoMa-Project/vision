/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include "ecto_rbo_grasping/EnvironmentalConstraintRegion.h"

EnvironmentalConstraintRegion::EnvironmentalConstraintRegion()
{
}

EnvironmentalConstraintRegion::EnvironmentalConstraintRegion(const tf::Transform& origin, const tf::Vector3& translation_boundaries, const tf::Vector3& orientation_boundaries_1)
    : origin(origin), translation_boundaries(translation_boundaries), orientation_boundaries_1(orientation_boundaries_1)
{
}

void EnvironmentalConstraintRegion::setOrigin(const tf::Transform& origin)
{
    this->origin = origin;
}

void EnvironmentalConstraintRegion::setTranslationBoundaries(const tf::Vector3& translation_boundaries)
{
    this->translation_boundaries = translation_boundaries;
}

void EnvironmentalConstraintRegion::setOrientationBoundaries(const tf::Vector3& orientation_boundaries)
{
    this->orientation_boundaries_1 = orientation_boundaries;
}

const tf::Transform& EnvironmentalConstraintRegion::getOrigin() const
{
    return this->origin;
}

const tf::Matrix3x3& EnvironmentalConstraintRegion::getOrientationOrigin() const
{
    return this->orientation_origin;
}

const tf::Vector3& EnvironmentalConstraintRegion::getOrientationBoundaries1() const
{
    return this->orientation_boundaries_1;
}

const tf::Vector3& EnvironmentalConstraintRegion::getOrientationBoundaries2() const
{
    return this->orientation_boundaries_2;
}

const Plane_3& EnvironmentalConstraintRegion::getPlane() const
{
    return this->plane;
}

//void EnvironmentalConstraintRegion::intersectRegionsOnSphere(const std::vector<Circular_arc_3>& edges1, const std::vector<Circular_arc_3>& edges2, std::vector<Circular_arc_3> result)
//{
//    for (size_t i = 0; i < edges1.size(); ++i)
//    {
//        // check if point inside or outside
//        edges1[i].source();

//        if (inside)
//        {
//            // add to intersecting region
//        }


//        for (size_t j = 0; j < edges2.size(); ++j)
//        {

//        }
//    }
//}

void EnvironmentalConstraintRegion::createAxisRegionOnSphere(const tf::Matrix3x3& axes, int axis, float width1, float width2, std::vector<Circular_arc_3> edges)
{
    // width and height can be between [-PI, PI[

    tf::Vector3 axis_vector = axes.getColumn(axis % 3);
    tf::Vector3 dim1 = axes.getColumn((axis + 1) % 3);
    tf::Vector3 dim2 = axes.getColumn((axis + 2) % 3);

    tf::Vector3 center_1 = dim1 * sin(width1);
    tf::Vector3 center_2 = -center_1;
    tf::Vector3 center_3 = dim2 * sin(width2);
    tf::Vector3 center_4 = -center_3;
    double radius_1 = cos(width1);
    double radius_3 = cos(width2);

    Circle_3 circle_1(Point_3(center_1.x(), center_1.y(), center_1.z()), radius_1 * radius_1, Vector_3(dim1.x(), dim1.y(), dim1.z()));
    Circle_3 circle_2(Point_3(center_2.x(), center_2.y(), center_2.z()), radius_1 * radius_1, Vector_3(-dim1.x(), -dim1.y(), -dim1.z()));
    Circle_3 circle_3(Point_3(center_3.x(), center_3.y(), center_3.z()), radius_3 * radius_3, Vector_3(dim2.x(), dim2.y(), dim2.z()));
    Circle_3 circle_4(Point_3(center_4.x(), center_4.y(), center_4.z()), radius_3 * radius_3, Vector_3(-dim2.x(), -dim2.y(), -dim2.z()));

    tf::Vector3 point_1 = center_1 + axis_vector * radius_1 * cos(width2) + dim2 * radius_1 * sin(width2);
    tf::Vector3 point_2 = center_1 + axis_vector * radius_1 * cos(width2) - dim2 * radius_1 * sin(width2);
    tf::Vector3 point_3 = center_2 + axis_vector * radius_1 * cos(width2) - dim2 * radius_1 * sin(width2);
    tf::Vector3 point_4 = center_2 + axis_vector * radius_1 * cos(width2) + dim2 * radius_1 * sin(width2);

    Circular_arc_3 arc12(circle_1, Circular_arc_point_3(Point_3(point_1.x(), point_1.y(), point_1.z())), Circular_arc_point_3(Point_3(point_2.x(), point_2.y(), point_2.z())));
    Circular_arc_3 arc34(circle_2, Circular_arc_point_3(Point_3(point_3.x(), point_3.y(), point_3.z())), Circular_arc_point_3(Point_3(point_4.x(), point_4.y(), point_4.z())));

    Circular_arc_3 arc23(circle_3, Circular_arc_point_3(Point_3(point_3.x(), point_3.y(), point_3.z())), Circular_arc_point_3(Point_3(point_4.x(), point_4.y(), point_4.z())));
    Circular_arc_3 arc41(circle_4, Circular_arc_point_3(Point_3(point_3.x(), point_3.y(), point_3.z())), Circular_arc_point_3(Point_3(point_4.x(), point_4.y(), point_4.z())));

    edges.push_back(arc12);
    edges.push_back(arc23);
    edges.push_back(arc34);
    edges.push_back(arc41);
}

bool EnvironmentalConstraintRegion::intersect(const EnvironmentalConstraintRegion& other, EnvironmentalConstraintRegion& result)
{
    // check if orientations overlap
    /*
    const tf::Matrix3x3& other_orientation = other.getOrientationOrigin();
    const tf::Vector3& other_orientation_boundaries_1 = other.getOrientationBoundaries1();
    const tf::Vector3& other_orientation_boundaries_2 = other.getOrientationBoundaries2();

    tf::Vector3 intersection_orientation_origin_cols[3];
    tf::Vector3 intersection_orientation_boundaries_1, intersection_orientation_boundaries_2;

    // intersect each dimension separately
    for (size_t i = 0; i < 3; ++i)
    {
        size_t index_1 = (i + 1) % 3;
        size_t index_2 = (i + 2) % 3;

        // intersect dimension i along dimension index_1
        tf::Vector3 cross_product = orientation_origin.getColumn(index_1).cross(other_orientation.getColumn(index_1));

        double arc_length = acos(cross_product.dot(orientation_origin.getColumn(i)));
        if (fabs(arc_length) > orientation_boundaries_1.x())
            return false;

        arc_length = acos(cross_product.dot(other_orientation.getColumn(i)));
        if (fabs(arc_length) > other_orientation_boundaries_1.x())
            return false;

        // do the same along dimension index_2
        cross_product = orientation_origin.getColumn(index_2).cross(other_orientation.getColumn(index_2));

        arc_length = acos(cross_product.dot(orientation_origin.getColumn(i)));
        if (fabs(arc_length) > orientation_boundaries_2.x())
            return false;

        arc_length = acos(cross_product.dot(other_orientation.getColumn(i)));
        if (fabs(arc_length) > other_orientation_boundaries_2.x())
            return false;

        intersection_orientation_origin_cols[i];
    }
    */

    // create four circular arcs for each axis and intersect them


    // check if translations overlap
    Segment_3 line_segment;
    CGAL::Object intersection_result = CGAL::intersection(plane, other.getPlane());
    if (!CGAL::assign(line_segment, intersection_result))
    {
        //            Plane_3 plane_segment;
        //            if( assign( il, result ) )
        //                    std::cout << "intersection between plane and line is a line --> l1 and l2 are parallel!" << std::endl;
        return false;
    }

    //

    return true;
}
