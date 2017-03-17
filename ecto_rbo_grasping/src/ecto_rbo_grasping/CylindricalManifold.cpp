/*
 * CylindricalManifold.cpp
 *
 *  Created on: Mar 4, 2013
 *      Author: clemens
 */

#include <iostream>
#include <math.h>
#include <nlopt.hpp>

#include "ecto_rbo_grasping/CylindricalManifold.h"

CylindricalManifold::CylindricalManifold(double radius, double height, double aperture)
  : radius(radius), height(height), aperture(aperture)
{
}

CylindricalManifold::~CylindricalManifold()
{
}

void CylindricalManifold::setOrigin(Eigen::Vector3f& translation, Eigen::Vector3f& cylindrical_axis, Eigen::Vector3f& x_axis)
{
//  origin.setOrigin(translation);
//  origin.setBasis(btMatrix3x3(x_axis, cylindrical_axis.cross(x_axis), cylindrical_axis));

  Eigen::Matrix3f rotation;
  rotation << x_axis, cylindrical_axis.cross(x_axis), cylindrical_axis;

  origin = Eigen::Translation3f(translation.x(), translation.y(), translation.z()) * rotation;
}

double CylindricalManifold::getRadius() const
{
  return radius;
}

double CylindricalManifold::getHeight() const
{
  return height;
}

double CylindricalManifold::getAperture() const
{
  return aperture;
}

double CylindricalManifold::getCartesianCoordinate(double phi, double z, Eigen::Vector3f& cartesian)
{
  double angular_error = (phi - aperture);

  cartesian = origin * Eigen::Vector3f(radius * cos(phi), radius * sin(z), z);

  return std::min(0.0, fabs(angular_error));
}

struct MyFuncData {
  double w1, w2;
  double orientation_x, orientation_y, orientation_z;
};

double myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{
  MyFuncData* data = static_cast<MyFuncData*>(my_func_data);
//    if (grad) {
//        grad[0] = 0.0;
//        grad[1] = 0.5 / sqrt(x[1]);
//    }

  // convert into cartesian coordinates: z is always zero
  double cos_x0 = cos(x[0]);
  double sin_x0 = sin(x[0]);

  //  double dist_cartesian = sqrt(x[0]*x[0] + x[1]*x[1]);
  // cylindrical: polar in one direction, cartesian in the other:
  double dist_polar_2 = 2.0 - 2.0 * cos_x0;
  double dist_cylindrical = sqrt(dist_polar_2 + x[1]*x[1]);

  // the dot product
  return data->w1 * (cos_x0 * data->orientation_x + sin_x0 * data->orientation_y) + data->w2 * (dist_cylindrical);
}

double CylindricalManifold::getCartesianCoordinate(Eigen::Vector3f& desired_normal, Eigen::Vector3f& resulting_normal, Eigen::Vector3f& cartesian)
{
  // transform desired normal orientation into the cylinder reference frame
  Eigen::Vector3f normal = origin.rotation().transpose() * desired_normal.normalized();

  nlopt::opt opt(nlopt::LN_COBYLA, 2);
  std::vector<double> lower_bounds(2), upper_bounds(2);
  lower_bounds[0] = -M_PI_2;
  lower_bounds[1] = -0.1;
  upper_bounds[0] = M_PI_2;
  upper_bounds[1] = +0.1;
  opt.set_lower_bounds(lower_bounds);
  opt.set_upper_bounds(upper_bounds);

  MyFuncData d;
  d.w1 = 1.0;
  d.w2 = 0.0;
  d.orientation_x = normal(0);
  d.orientation_y = normal(1);
  d.orientation_z = normal(2);
  opt.set_min_objective(myfunc, &d);

  opt.set_xtol_rel(1e-4);

  std::vector<double> x(2);
  x[0] = 0.0;
  x[1] = 0.0;
  double minf;
  nlopt::result result = opt.optimize(x, minf);

  std::cout << "cylindrical optimization result: " << minf << " at " << x[0] << ", " << x[1] << std::endl;

  // convert from cylindrical coordinates to cartesian
  cartesian(0) = radius * cos(x[0]);
  cartesian(1) = radius * sin(x[0]);
  cartesian(2) = x[1];

  resulting_normal(0) = -cos(x[0]);
  resulting_normal(1) = -sin(x[0]);
  resulting_normal(2) = 0;

  // transform to world frame
  cartesian = origin * cartesian;
  resulting_normal = origin.rotation() * resulting_normal;

/*
  // project normal onto x-y-plane (all normals on the cylinder lie in a plane)
  Eigen::Vector3f cylinder_axis = origin.rotation().col(2);
  Eigen::Vector3f flattened = normal - normal.dot(cylinder_axis) * cylinder_axis;

  // check difference between flattened and origin
  Eigen::Vector3f origin_on_cylinder = origin.rotation().col(0);
  double angular_error = std::acos(origin_on_cylinder.dot(flattened));

  std::cout << "angular error " << angular_error << std::endl;

  /*
  // return the pose that is optimal with respect to the orientation error
  if (fabs(angular_error) > aperture)
  {
    resulting_normal = flattened;
    Eigen::AngleAxisf rotate(aperture, cylinder_axis);
    cartesian = rotate * origin_on_cylinder;

    std::cout << "CHANGING SHHHHITITTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT!" << std::endl;

    angular_error = fabs(angular_error) - aperture;
  }
  else
  {
    resulting_normal = flattened;
    angular_error = 0;

    cartesian = flattened;
//  }

  cartesian *= radius;
  cartesian = origin.translation() + cartesian;

  return angular_error;
  */
}

void CylindricalManifold::getCartesianCoordinateDim(const Eigen::Vector3f& positive_dimension, Eigen::Vector3f& cartesian)
{
  Eigen::Vector3f cylinder_axis(0, 0, 1);

  double dot_product = positive_dimension.dot(cylinder_axis);
  if (fabs(dot_product) < 1e-5)
  {
    cartesian = Eigen::Vector3f(1, 0, 0);
  }
  else if (dot_product > 0)
  {
    cartesian = Eigen::Vector3f(1, 0, +height);
  }
  else
  {
    cartesian = Eigen::Vector3f(1, 0, -height);
  }

  cartesian = origin * cartesian;
}
