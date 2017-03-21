/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_rbo_grasping/PoseSet.h>

#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>

#include <CGAL/Gmpz.h>
#include <CGAL/Extended_homogeneous.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_3.h>

#include "Wm5IntrSegment3Box3.h"
#include "Wm5IntrBox3Box3.h"
#include "Wm5ContMinBox3.h"

#include <ecto_rbo_grasping/gdiam.hpp>

//typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
//typedef CGAL::Extended_homogeneous<CGAL::Gmpz>  Kernel;
typedef CGAL::Extended_cartesian< CGAL::Lazy_exact_nt<CGAL::Gmpq> > Kernel;
typedef CGAL::Nef_polyhedron_3<Kernel>  Nef_polyhedron;
typedef Nef_polyhedron::Vertex_const_iterator Vertex_const_iterator;
typedef Nef_polyhedron::Plane_3  Plane_3;
typedef Nef_polyhedron::Point_3  Point_3;
typedef Nef_polyhedron::Vector_3  Vector_3;

namespace posesets
{

PoseSet::PoseSet(const tf::Transform& origin) :
    origin(origin)
{
}

PoseSet::~PoseSet()
{
}

void PoseSet::updateBoxVertices(Wm5::Vector3d box_vertices[8], Wm5::Segment3d edges[12]) const
{
    Wm5::Vector3d origin_a(this->origin.getOrigin().x(), this->origin.getOrigin().y(), this->origin.getOrigin().z());
    //    Wm5::Vector3d dim_a[3] = { Wm5::Vector3d(a.getBasis().getRow(0).x(), a.getBasis().getRow(0).y(), a.getBasis().getRow(0).z()),
    //                               Wm5::Vector3d(a.getBasis().getRow(1).x(), a.getBasis().getRow(1).y(), a.getBasis().getRow(1).z()),
    //                               Wm5::Vector3d(a.getBasis().getRow(2).x(), a.getBasis().getRow(2).y(), a.getBasis().getRow(2).z()) };
    Wm5::Vector3d dim_a[3] = { Wm5::Vector3d(this->origin.getBasis().getColumn(0).x(), this->origin.getBasis().getColumn(0).y(), this->origin.getBasis().getColumn(0).z()),
                               Wm5::Vector3d(this->origin.getBasis().getColumn(1).x(), this->origin.getBasis().getColumn(1).y(), this->origin.getBasis().getColumn(1).z()),
                               Wm5::Vector3d(this->origin.getBasis().getColumn(2).x(), this->origin.getBasis().getColumn(2).y(), this->origin.getBasis().getColumn(2).z()) };
    double size_a[3] = {0.5 * this->getPositions()[0], 0.5 * this->getPositions()[1], 0.5 * this->getPositions()[2]};

    Wm5::Box3d box_a(origin_a, dim_a, size_a);
    box_a.ComputeVertices(box_vertices);

    edges[0] = Wm5::Segment3d(box_vertices[0], box_vertices[1]);
    edges[1] = Wm5::Segment3d(box_vertices[0], box_vertices[3]);
    edges[2] = Wm5::Segment3d(box_vertices[0], box_vertices[4]);
    edges[3] = Wm5::Segment3d(box_vertices[1], box_vertices[2]);
    edges[4] = Wm5::Segment3d(box_vertices[1], box_vertices[5]);
    edges[5] = Wm5::Segment3d(box_vertices[2], box_vertices[6]);
    edges[6] = Wm5::Segment3d(box_vertices[2], box_vertices[3]);
    edges[7] = Wm5::Segment3d(box_vertices[3], box_vertices[7]);
    edges[8] = Wm5::Segment3d(box_vertices[4], box_vertices[7]);
    edges[9] = Wm5::Segment3d(box_vertices[4], box_vertices[5]);
    edges[10] = Wm5::Segment3d(box_vertices[5], box_vertices[6]);
    edges[11] = Wm5::Segment3d(box_vertices[6], box_vertices[7]);
}

void PoseSet::setOrigin(const tf::Transform& origin)
{
    this->origin = origin;
}

void PoseSet::setOrientations(const OrientationSet& orientations)
{
    this->orientations = orientations;
}

void PoseSet::setPositions(const tf::Vector3& positions)
{
    this->positions = positions;
}

const tf::Transform& PoseSet::getOrigin() const
{
    return origin;
}

OrientationSet& PoseSet::getOrientations()
{
    return orientations;
}

const OrientationSet& PoseSet::getOrientations() const
{
    return orientations;
}

const tf::Vector3& PoseSet::getPositions() const
{
    return positions;
}

Wm5::Box3d PoseSet::getWm5Box3d() const
{
    Wm5::Vector3d origin_a(origin.getOrigin().x(), origin.getOrigin().y(), origin.getOrigin().z());
    Wm5::Vector3d dim_a[3] = { Wm5::Vector3d(origin.getBasis().getColumn(0).x(), origin.getBasis().getColumn(0).y(), origin.getBasis().getColumn(0).z()),
                               Wm5::Vector3d(origin.getBasis().getColumn(1).x(), origin.getBasis().getColumn(1).y(), origin.getBasis().getColumn(1).z()),
                               Wm5::Vector3d(origin.getBasis().getColumn(2).x(), origin.getBasis().getColumn(2).y(), origin.getBasis().getColumn(2).z()) };
    double size_a[3] = {0.5 * positions[0], 0.5 * positions[1], 0.5 * positions[2]};

    return Wm5::Box3d(origin_a, dim_a, size_a);
}

bool PoseSet::isIntersecting(const PoseSet& other) const
{
//    if (this->orientations.getIntersection(other.getOrientations()).empty())
    if (!this->orientations.hasIntersection(other.getOrientations()))
    {
//        std::cout << "orientation false " << std::endl;
        return false;
    }

    Wm5::IntrBox3Box3d intersector(this->getWm5Box3d(), other.getWm5Box3d());

//    if (!intersector.Test())
//        std::cout << "position false " << std::endl;

    return intersector.Test();
}

bool PoseSet::intersect(const PoseSet& other)
{
    this->orientations.intersect(other.getOrientations());

    if (this->orientations.empty())
        return false;

    return intersectBoxesGeometricTools(this->origin, this->positions, other.getOrigin(), other.getPositions(), this->origin, this->positions);

    /*
    if (intersectBoxes(this->origin, this->positions, other.getOrigin(), other.getPositions()))
    {
        ROS_INFO("Intersection! (maybe)");
//        this->origin.setOrigin(origin.getOrigin().lerp(other.getOrigin().getOrigin(), 0.5));
//        this->positions.setMin(other.getPositions());
    }
    else {
        ROS_INFO("They DONT intersect!");
        this->origin.setIdentity();
        this->positions.setZero();
    }
    */
}


bool PoseSet::intersectBoxesGeometricTools(const tf::Transform& a, const tf::Vector3& extents_a, const tf::Transform& b, const tf::Vector3& extents_b, tf::Transform& n, tf::Vector3& extents_n)
{
    Wm5::Vector3d origin_a(a.getOrigin().x(), a.getOrigin().y(), a.getOrigin().z());
//    Wm5::Vector3d dim_a[3] = { Wm5::Vector3d(a.getBasis().getRow(0).x(), a.getBasis().getRow(0).y(), a.getBasis().getRow(0).z()),
//                               Wm5::Vector3d(a.getBasis().getRow(1).x(), a.getBasis().getRow(1).y(), a.getBasis().getRow(1).z()),
//                               Wm5::Vector3d(a.getBasis().getRow(2).x(), a.getBasis().getRow(2).y(), a.getBasis().getRow(2).z()) };
    Wm5::Vector3d dim_a[3] = { Wm5::Vector3d(a.getBasis().getColumn(0).x(), a.getBasis().getColumn(0).y(), a.getBasis().getColumn(0).z()),
                               Wm5::Vector3d(a.getBasis().getColumn(1).x(), a.getBasis().getColumn(1).y(), a.getBasis().getColumn(1).z()),
                               Wm5::Vector3d(a.getBasis().getColumn(2).x(), a.getBasis().getColumn(2).y(), a.getBasis().getColumn(2).z()) };
    double size_a[3] = {0.5 * extents_a[0], 0.5 * extents_a[1], 0.5 * extents_a[2]};

    Wm5::Vector3d origin_b(b.getOrigin().x(), b.getOrigin().y(), b.getOrigin().z());
//    Wm5::Vector3d dim_b[3] = { Wm5::Vector3d(b.getBasis().getRow(0).x(), b.getBasis().getRow(0).y(), b.getBasis().getRow(0).z()),
//                               Wm5::Vector3d(b.getBasis().getRow(1).x(), b.getBasis().getRow(1).y(), b.getBasis().getRow(1).z()),
//                               Wm5::Vector3d(b.getBasis().getRow(2).x(), b.getBasis().getRow(2).y(), b.getBasis().getRow(2).z()) };
    Wm5::Vector3d dim_b[3] = { Wm5::Vector3d(b.getBasis().getColumn(0).x(), b.getBasis().getColumn(0).y(), b.getBasis().getColumn(0).z()),
                               Wm5::Vector3d(b.getBasis().getColumn(1).x(), b.getBasis().getColumn(1).y(), b.getBasis().getColumn(1).z()),
                               Wm5::Vector3d(b.getBasis().getColumn(2).x(), b.getBasis().getColumn(2).y(), b.getBasis().getColumn(2).z()) };
    double size_b[3] = {0.5 * extents_b[0], 0.5 * extents_b[1], 0.5 * extents_b[2]};

    Wm5::Box3d box_a(origin_a, dim_a, size_a);
    Wm5::Box3d box_b(origin_b, dim_b, size_b);
    Wm5::IntrBox3Box3d intersector(box_a, box_b);

    if (!intersector.Test())
    {
        //ROS_INFO("NO intersection.");
        n.setIdentity();
        extents_n.setZero();
        return false;
    }

    // do static check differently:
    // check intersections between 12 edges (segments) of one box with the other box
    intersection_points.clear();
//    std::vector<Wm5::Vector3d> intersection_points;
    for (int l = 0; l < 2; ++l)
    {
        const Wm5::Box3d& box = (l == 0) ? box_a : box_b;
        const Wm5::Box3d& other_box = (l == 1) ? box_a : box_b;

        Wm5::Vector3d vertices[8];
        box.ComputeVertices(vertices);

//        if (l == 0)
//            for (int k = 0; k < 8; ++k)
//                this->box_vertices[k] = vertices[k];

        Wm5::Segment3d edges[12] = {
            Wm5::Segment3d(vertices[0], vertices[1]),
            Wm5::Segment3d(vertices[0], vertices[3]),
            Wm5::Segment3d(vertices[0], vertices[4]),
            Wm5::Segment3d(vertices[1], vertices[2]),
            Wm5::Segment3d(vertices[1], vertices[5]),
            Wm5::Segment3d(vertices[2], vertices[6]),
            Wm5::Segment3d(vertices[2], vertices[3]),
            Wm5::Segment3d(vertices[3], vertices[7]),
            Wm5::Segment3d(vertices[4], vertices[7]),
            Wm5::Segment3d(vertices[4], vertices[5]),
            Wm5::Segment3d(vertices[5], vertices[6]),
            Wm5::Segment3d(vertices[6], vertices[7])
        };

        for (int i = 0; i < 12; ++i)
        {
            Wm5::IntrSegment3Box3d intersector2(edges[i], other_box, true);
            if (!intersector2.Find())
                continue;

            int no_intersections = intersector2.GetQuantity();

    //        std::cout << "Found: " << no_intersections << std::endl;
            for (int j = 0; j < no_intersections; ++j)
            {
//                bool already_in_there = false;
                const Wm5::Vector3d& candidate = intersector2.GetPoint(j);
//                // check if for some reasons this point is already in there
//                for (int k = 0; k < intersection_points.size(); ++k)
//                {
//                    if (fabs(candidate.X() - intersection_points[k].X()) < 0.001f ||
//                        fabs(candidate.Y() - intersection_points[k].Y()) < 0.001f ||
//                        fabs(candidate.Z() - intersection_points[k].Z()) < 0.001f)
//                    {
//                        already_in_there = true;
//                        break;
//                    }
//                }
//                if (!already_in_there)
                    intersection_points.push_back(intersector2.GetPoint(j));
            }
        }
    }
    
    /*
    std::cout << "In Total: " << intersection_points.size() << std::endl;
    for (std::vector<Wm5::Vector3d>::iterator it = intersection_points.begin(); it != intersection_points.end(); ++it)
    {
        std::cout << " (" << it->X() << " " << it->Y() << " " << it->Z() << ")";
    }
    std::cout << std::endl;
    */
    
    if (intersection_points.size() < 4)
    {
        n.setIdentity();
        extents_n.setZero();
        return false;
    }

//    Wm5::Box3d minBox = Wm5::MinBox3<double>((int) intersection_points.size(), &intersection_points[0], 0.001, Wm5::Query::QT_REAL);

//    n.setBasis(tf::Matrix3x3(minBox.Axis[0].X(), minBox.Axis[0].Y(), minBox.Axis[0].Z(),
//                             minBox.Axis[1].X(), minBox.Axis[1].Y(), minBox.Axis[1].Z(),
//                             minBox.Axis[2].X(), minBox.Axis[2].Y(), minBox.Axis[2].Z()));

//    n.setOrigin(tf::Vector3(minBox.Center.X(), minBox.Center.Y(), minBox.Center.Z()));

//    std::cout << "New Box extents: " << extents_n[0] << " " << extents_n[1] << " " << extents_n[2] << std::endl;

//    extents_n.setX(minBox.Extent[0]);
//    extents_n.setY(minBox.Extent[1]);
//    extents_n.setZ(minBox.Extent[2]);

//    if (extents_n[0] == 0 || extents_n[1] == 0 || extents_n[2] == 0)
//    {
//        n.setIdentity();
//        extents_n.setZero();
//        return false;
//    }


//    intersector.Find(0.0, Wm5::Vector3d::ZERO, Wm5::Vector3d::ZERO);
//    intersector.Find(0.1, Wm5::Vector3d(0.1, 0.1, 0.1), Wm5::Vector3d(0.1, 0.1, 0.1));
//    int number_of_intersection_points = intersector.GetQuantity();

//    std::cout << "Number of intersection points: " << number_of_intersection_points << std::endl;
//    if (number_of_intersection_points == 0)
//    {
//        n.setIdentity();
//        extents_n.setZero();
//        return false;
//    }

    gdiam_real* points;
    points = (gdiam_point)malloc( sizeof( gdiam_point_t ) * intersection_points.size());

    for (int i = 0; i < intersection_points.size(); ++i)
    {
        const Wm5::Vector3d& p = intersection_points[i];
        points[i * 3 + 0] = p.X();
        points[i * 3 + 1] = p.Y();
        points[i * 3 + 2] = p.Z();
//        std::cout << "HA: " << p.X() << " " << p.Y() << " " << p.Z() << std::endl;
    }

    gdiam_point* pnt_arr;
    gdiam_bbox bb;
    pnt_arr = gdiam_convert( (gdiam_real *)points, intersection_points.size());

//    printf( "Computing a tight-fitting bounding box of the point-set\n" );
//    bb = gdiam_approx_mvbb_grid_sample(pnt_arr, nefI.number_of_vertices(), 5, 400 );
    bb = gdiam_approx_mvbb(pnt_arr, intersection_points.size(), 0.01f);

    //printf( "Resulting bounding box:\n" );
    bb.dump();

    gdiam_point bb_dir0 = bb.get_dir(0);
    gdiam_point bb_dir1 = bb.get_dir(1);
    gdiam_point bb_dir2 = bb.get_dir(2);

    n.setBasis(tf::Matrix3x3(bb_dir0[0], bb_dir1[0], bb_dir2[0],
                             bb_dir0[1], bb_dir1[1], bb_dir2[1],
                             bb_dir0[2], bb_dir1[2], bb_dir2[2]));

    double x, y, z;
    bb.get_vertex(0.5, 0.5, 0.5, &x, &y, &z);
    n.setOrigin(tf::Vector3(x, y, z));

    extents_n.setX(bb.get_len(0) * 1.f);
    extents_n.setY(bb.get_len(1) * 1.f);
    extents_n.setZ(bb.get_len(2) * 1.f);

//    if (extents_n[0] == 0 || extents_n[1] == 0 || extents_n[2] == 0)
    if (bb.volume() == 0)
    {
        //ROS_INFO("Too Small!");
        n.setIdentity();
        extents_n.setZero();
        return false;
    }

    return true;
}

void PoseSet::intersectBoxesCGAL(const tf::Transform& a, const tf::Vector3& extents_a, const tf::Transform& b, const tf::Vector3& extents_b, tf::Transform& n, tf::Vector3& extents_n)
{
    // generate a Nef Polyhedron for box A
    Point_3 origin_a(a.getOrigin().x(), a.getOrigin().x(), a.getOrigin().x());
    Vector_3 dim1_a(a.getBasis().getRow(0).x(), a.getBasis().getRow(0).y(), a.getBasis().getRow(0).z());
    Vector_3 dim2_a(a.getBasis().getRow(1).x(), a.getBasis().getRow(1).y(), a.getBasis().getRow(1).z());
    Vector_3 dim3_a(a.getBasis().getRow(2).x(), a.getBasis().getRow(2).y(), a.getBasis().getRow(2).z());
    Nef_polyhedron NA1(Plane_3(origin_a + dim1_a * extents_a[0], dim1_a), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NA2(Plane_3(origin_a - dim1_a * extents_a[0], -dim1_a), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NA3(Plane_3(origin_a + dim2_a * extents_a[1], dim2_a), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NA4(Plane_3(origin_a - dim2_a * extents_a[1], -dim2_a), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NA5(Plane_3(origin_a + dim3_a * extents_a[2], dim3_a), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NA6(Plane_3(origin_a - dim3_a * extents_a[2], -dim3_a), Nef_polyhedron::INCLUDED);

//    Nef_polyhedron nefA = NA1 * NA2 * NA3 * NA4 * NA5 * NA6;

    // generate a Nef Polyhedron for box B
    Point_3 origin_b(b.getOrigin().x(), b.getOrigin().x(), b.getOrigin().x());
    Vector_3 dim1_b(b.getBasis().getRow(0).x(), b.getBasis().getRow(0).y(), b.getBasis().getRow(0).z());
    Vector_3 dim2_b(b.getBasis().getRow(1).x(), b.getBasis().getRow(1).y(), b.getBasis().getRow(1).z());
    Vector_3 dim3_b(b.getBasis().getRow(2).x(), b.getBasis().getRow(2).y(), b.getBasis().getRow(2).z());
    Nef_polyhedron NB1(Plane_3(origin_b + dim1_b * extents_b[0], dim1_b), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NB2(Plane_3(origin_b - dim1_b * extents_b[0], -dim1_b), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NB3(Plane_3(origin_b + dim2_b * extents_b[1], dim2_b), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NB4(Plane_3(origin_b - dim2_b * extents_b[1], -dim2_b), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NB5(Plane_3(origin_b + dim3_b * extents_b[2], dim3_b), Nef_polyhedron::INCLUDED);
    Nef_polyhedron NB6(Plane_3(origin_b - dim3_b * extents_b[2], -dim3_b), Nef_polyhedron::INCLUDED);

//    Nef_polyhedron nefB = NB1 * NB2 * NB3 * NB4 * NB5 * NB6;

//    Nef_polyhedron nefI = nefA.intersection(nefB);
    Nef_polyhedron nefI = NA1 * NA2 * NA3 * NA4 * NA5 * NA6 * NB1 * NB2 * NB3 * NB4 * NB5 * NB6;

    /*
    std::cout << "properties: " << std::endl;
    std::cout << "is_simple: " << nefI.is_simple() << std::endl;
    std::cout << "is_bounded: " << nefI.is_bounded() << std::endl;
//    std::cout << "is_convex: " << nefI.is_convex() << std::endl;
    std::cout << "vertices: " << nefI.number_of_vertices() << std::endl;
    std::cout << "is_empty: " << nefI.is_empty() << std::endl;
    */
    
    if (nefI.is_empty())
        return;

    Vertex_const_iterator it;
//    gdiam_point* pnt_arr;// = new gdiam_point[nefI.number_of_vertices()];
//    pnt_arr = (gdiam_point *)malloc( sizeof( gdiam_point ) * nefI.number_of_vertices());
    gdiam_real* points;
    points = (gdiam_point)malloc( sizeof( gdiam_point_t ) * nefI.number_of_vertices());

    size_t i = 0;
    CGAL_forall_vertices(it, nefI)
    {
        //std::cout << CGAL::to_double(it->point().x()) << " HAH " << CGAL::to_double(it->point().y()) << " " << CGAL::to_double(it->point().z()) << std::endl;
//        pnt_init((gdiam_point)(pnt_arr[i]), 0, 0, 0);
        points[i * 3 + 0] = CGAL::to_double(it->point().x());
        points[i * 3 + 1] = CGAL::to_double(it->point().y());
        points[i * 3 + 2] = CGAL::to_double(it->point().z());
//        pnt_init(pnt_arr[i], CGAL::to_double(it->point().x()), CGAL::to_double(it->point().y()), CGAL::to_double(it->point().z()));
        ++i;
    }

    gdiam_point* pnt_arr;
    gdiam_bbox bb;
    pnt_arr = gdiam_convert( (gdiam_real *)points, nefI.number_of_vertices());

//    printf( "Computing a tight-fitting bounding box of the point-set\n" );
//    bb = gdiam_approx_mvbb_grid_sample(pnt_arr, nefI.number_of_vertices(), 5, 400 );
    bb = gdiam_approx_mvbb(pnt_arr, nefI.number_of_vertices(), 0.1f);

    //printf( "Resulting bounding box:\n" );
    bb.dump();

    gdiam_point bb_dir0 = bb.get_dir(0);
    gdiam_point bb_dir1 = bb.get_dir(1);
    gdiam_point bb_dir2 = bb.get_dir(2);

    n.setBasis(tf::Matrix3x3(bb_dir0[0], bb_dir0[1], bb_dir0[2],
                             bb_dir1[0], bb_dir1[1], bb_dir1[2],
                             bb_dir2[0], bb_dir2[1], bb_dir2[2]));

    double x, y, z;
    bb.get_vertex(0.5, 0.5, 0.5, &x, &y, &z);
    n.setOrigin(tf::Vector3(x, y, z));

    extents_n.setX(bb.get_len(0) * 0.5f);
    extents_n.setY(bb.get_len(1) * 0.5f);
    extents_n.setZ(bb.get_len(2) * 0.5f);
}

void PoseSet::intersectBoxes(const tf::Transform& a, const tf::Vector3& extents_a, const tf::Transform& b, const tf::Vector3& extents_b, tf::Transform& n, tf::Vector3& extents_n)
{
//    tf::Transform b_a = a.inverseTimes(b);
//    tf::Transform a_b = b.inverseTimes(a);

//    // do aabb-intersection along axes of a
//    tf::Matrix3x3 all_dot_products = a.getBasis().transposeTimes(b_a.getBasis());
//    tf::Vector3 extents_b_a(all_dot_products.getRow(0)[all_dot_products.getRow(0).maxAxis()],
//                            all_dot_products.getRow(1)[all_dot_products.getRow(1).maxAxis()],
//                            all_dot_products.getRow(2)[all_dot_products.getRow(2).maxAxis()]);
//    extents_b_a *= extents_b;

//    // do aabb-intersection along axes of b
////    all_dot_products = b.getBasis().transposeTimes(a_b.getBasis());
////    tf::Vector3 extents_a_b(all_dot_products.getRow(0)[all_dot_products.getRow(0).maxAxis()],
////                            all_dot_products.getRow(1)[all_dot_products.getRow(1).maxAxis()],
////                            all_dot_products.getRow(2)[all_dot_products.getRow(2).maxAxis()]);
////    extents_a_b *= extents_a;

////    if (extents_b_a.length2() < extents_a_b.length2())
////    {
//        // a is the reference frame
//        Eigen::Vector3d extents, origin;
//        tf::vectorTFToEigen(extents_a, extents);
//        tf::vectorTFToEigen(a.getOrigin(), origin);
//        Eigen::AlignedBox<float, 3> box_a(origin + extents, origin - extents);

////        Eigen::AlignedBox<float, 3> box_b(v, v);

////        Eigen::AlignedBox<float, 3> box_i = box_a.intersection(box_b);

////        tf::vectorEigenToTF(box_i.center(), n.getOrigin());
////        tf::vectorEigenToTF(box_i.max(), extents_n);
////        n.setRotation(a.getRotation());
////    }
////    else {
////        extents_n = (extents_b - extents_a_b).absolute();
////        n.setOrigin(b.getOrigin() + extents_b - extents_n * 0.5);
////        n.setRotation(b.getRotation());
////    }
}

bool PoseSet::intersectBoxes(const tf::Transform& a, const tf::Vector3& extents_a, const tf::Transform& b, const tf::Vector3& extents_b)
{
    tf::Matrix3x3 A = a.getBasis();
    tf::Matrix3x3 B = b.getBasis();

    //translation, in parent frame
    tf::Vector3 v = b.getOrigin() - a.getOrigin();
    //translation, in A's frame
    tf::Vector3 T(v.dot(A.getColumn(0)), v.dot(A.getColumn(1)), v.dot(A.getColumn(2)));

    //B's basis with respect to A's local frame
    tf::Matrix3x3 R = A.transposeTimes(B);
    float ra, rb, t;
    long i, k;

    //calculate rotation matrix
//        for( i=0 ; i<3 ; i++ )
//            for( k=0 ; k<3 ; k++ )
//                R[i][k] = A.getColumn(i).dot(B.getColumn(k));
    /*ALGORITHM: Use the separating axis test for all 15 potential
separating axes. If a separating axis could not be found, the two
boxes overlap. */

    //A's basis vectors
    for( i=0 ; i<3 ; i++ )
    {
        ra = extents_a[i];
        rb = extents_b[0]*fabs(R[i][0]) + extents_b[1]*fabs(R[i][1]) + extents_b[2]*fabs(R[i][2]);

        t = fabs( T[i] );

        if( t > ra + rb )
            return false;
    }

    //B's basis vectors
    for( k=0 ; k<3 ; k++ )
    {
        ra = extents_a[0]*fabs(R[0][k]) + extents_a[1]*fabs(R[1][k]) + extents_a[2]*fabs(R[2][k]);
        rb = extents_b[k];

        t = fabs( T[0]*R[0][k] + T[1]*R[1][k] + T[2]*R[2][k] );

        if( t > ra + rb )
            return false;
    }

    //9 cross products

    //L = A0 x B0
    ra = extents_a[1]*fabs(R[2][0]) + extents_a[2]*fabs(R[1][0]);

    rb = extents_b[1]*fabs(R[0][2]) + extents_b[2]*fabs(R[0][1]);

    t = fabs( T[2]*R[1][0] - T[1]*R[2][0] );

    if( t > ra + rb )
        return false;

    //L = A0 x B1
    ra = extents_a[1]*fabs(R[2][1]) + extents_a[2]*fabs(R[1][1]);

    rb = extents_b[0]*fabs(R[0][2]) + extents_b[2]*fabs(R[0][0]);

    t = fabs( T[2]*R[1][1] - T[1]*R[2][1] );

    if( t > ra + rb )
        return false;

    //L = A0 x B2
    ra = extents_a[1]*fabs(R[2][2]) + extents_a[2]*fabs(R[1][2]);

    rb = extents_b[0]*fabs(R[0][1]) + extents_b[1]*fabs(R[0][0]);

    t = fabs( T[2]*R[1][2] - T[1]*R[2][2] );

    if( t > ra + rb )
        return false;

    //L = A1 x B0
    ra = extents_a[0]*fabs(R[2][0]) + extents_a[2]*fabs(R[0][0]);

    rb = extents_b[1]*fabs(R[1][2]) + extents_b[2]*fabs(R[1][1]);

    t = fabs( T[0]*R[2][0] - T[2]*R[0][0] );

    if( t > ra + rb )
        return false;

    //L = A1 x B1
    ra = extents_a[0]*fabs(R[2][1]) + extents_a[2]*fabs(R[0][1]);

    rb = extents_b[0]*fabs(R[1][2]) + extents_b[2]*fabs(R[1][0]);

    t = fabs( T[0]*R[2][1] - T[2]*R[0][1] );

    if( t > ra + rb )
        return false;

    //L = A1 x B2
    ra = extents_a[0]*fabs(R[2][2]) + extents_a[2]*fabs(R[0][2]);

    rb = extents_b[0]*fabs(R[1][1]) + extents_b[1]*fabs(R[1][0]);

    t = fabs( T[0]*R[2][2] - T[2]*R[0][2] );

    if( t > ra + rb )
        return false;

    //L = A2 x B0
    ra = extents_a[0]*fabs(R[1][0]) + extents_a[1]*fabs(R[0][0]);

    rb = extents_b[1]*fabs(R[2][2]) + extents_b[2]*fabs(R[2][1]);

    t = fabs( T[1]*R[0][0] - T[0]*R[1][0] );

    if( t > ra + rb )
        return false;

    //L = A2 x B1
    ra = extents_a[0]*fabs(R[1][1]) + extents_a[1]*fabs(R[0][1]);

    rb = extents_b[0] *fabs(R[2][2]) + extents_b[2]*fabs(R[2][0]);

    t = fabs( T[1]*R[0][1] - T[0]*R[1][1] );

    if( t > ra + rb )
        return false;

    //L = A2 x B2
    ra = extents_a[0]*fabs(R[1][2]) + extents_a[1]*fabs(R[0][2]);

    rb = extents_b[0]*fabs(R[2][1]) + extents_b[1]*fabs(R[2][0]);

    t = fabs( T[1]*R[0][2] - T[0]*R[1][2] );

    if( t > ra + rb )
        return false;

    /*no separating axis found, the two boxes overlap */

    return true;
}

}
