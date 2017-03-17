/*
 * SphericalManifold.cpp
 *
 *  Created on: Mar 4, 2013
 *      Author: clemens
 */

#include <iostream>
#include <math.h>
#include <nlopt.hpp>

#include "ecto_rbo_grasping/SphericalManifold.h"

SphericalManifold::SphericalManifold(double radius, double aperture)
  : radius(radius), aperture(aperture)
{
}

SphericalManifold::~SphericalManifold()
{
}

double SphericalManifold::getRadius() const
{
  return radius;
}

double SphericalManifold::getAperture() const
{
  return aperture;
}

void SphericalManifold::setOrigin(Eigen::Vector3f& translation, Eigen::Vector3f& middle_finger, Eigen::Vector3f& retreat)
{
  // origin is along Z !
  // positive inclination (+90deg) is along X ==> rotation around +Y
  // positive azimuth (+90deg) is along Z (gimbal lock) ==> rotation around +Z

  Eigen::Matrix3f rotation;
  rotation << middle_finger, retreat.cross(middle_finger), retreat;

  origin = Eigen::Translation3f(translation.x(), translation.y(), translation.z()) * rotation;
}

double SphericalManifold::getCartesianCoordinate(double inclination, double azimuth, Eigen::Vector3f& cartesian)
{
  cartesian = origin * Eigen::Vector3f(radius * sin(inclination) * cos(azimuth),
                                       radius * sin(inclination) * sin(azimuth),
                                       radius * cos(inclination));

  return std::max(fabs(inclination - aperture), fabs(azimuth - aperture));
}
struct MyFuncData2 {
  double w1, w2;
  double orientation_x, orientation_y, orientation_z;
};

double myfunc2(unsigned n, const double *x, double *grad, void *my_func_data)
{
  MyFuncData2* data = static_cast<MyFuncData2*>(my_func_data);
//    if (grad) {
//        grad[0] = 0.0;
//        grad[1] = 0.5 / sqrt(x[1]);
//    }

  // convert into cartesian coordinates: z is always zero
  double cos_x0 = cos(x[0]);
  double sin_x0 = sin(x[0]);

  double n0 = -sin_x0 * cos(x[1]);
  double n1 = -sin_x0 * sin(x[1]);
  double n2 = -cos_x0;

  //  double dist_cartesian = sqrt(x[0]*x[0] + x[1]*x[1]);
  // spherical: polar in both directions, from the origin
  double dist_spherical = acos(cos_x0 * fabs(x[1]));

  // the negative dot product
  return -data->w1 * fabs(n0 * data->orientation_x + n1 * data->orientation_y + n2 * data->orientation_z) + data->w2 * (dist_spherical);
}

double SphericalManifold::getCartesianCoordinate(Eigen::Vector3f& desired_normal, Eigen::Vector3f& resulting_normal, Eigen::Vector3f& cartesian)
{
  // transform desired normal orientation into the spherical reference frame

  std::cout << "before: " << desired_normal(0) << " " << desired_normal(1) << " " << desired_normal(2) << std::endl;
  Eigen::Vector3f normal = origin.rotation().transpose() * desired_normal.normalized();
  std::cout << "after: " << normal(0) << " " << normal(1) << " " << normal(2) << std::endl;

  nlopt::opt opt(nlopt::LN_COBYLA, 2);
  std::vector<double> lower_bounds(2), upper_bounds(2);
  lower_bounds[0] = -M_PI_4;
  lower_bounds[1] = -M_PI_4;
  upper_bounds[0] = +M_PI_4;
  upper_bounds[1] = +M_PI_4;
  opt.set_lower_bounds(lower_bounds);
  opt.set_upper_bounds(upper_bounds);

  MyFuncData2 d;
  d.w1 = 1.0;
  d.w2 = 0.07;
  d.orientation_x = normal(0);
  d.orientation_y = normal(1);
  d.orientation_z = normal(2);
  opt.set_min_objective(myfunc2, &d);

  opt.set_xtol_rel(1e-4);

  std::vector<double> x(2);
  x[0] = 0.0;
  x[1] = 0.0;
  double minf;
  nlopt::result result = opt.optimize(x, minf);

  std::cout << "spherical optimization result: " << minf << " at " << x[0] << ", " << x[1] << std::endl;

  // convert from spherical coordinates to cartesian
  cartesian(0) = radius * sin(x[0]) * cos(x[1]);
  cartesian(1) = radius * sin(x[0]) * sin(x[1]);
  cartesian(2) = radius * cos(x[0]);

  resulting_normal(0) = -sin(x[0]) * cos(x[1]);
  resulting_normal(1) = -sin(x[0]) * sin(x[1]);
  resulting_normal(2) = -cos(x[0]);

  // transform to world frame
  cartesian = origin * cartesian;
  resulting_normal = origin.rotation() * resulting_normal;

  return 0.0;
}

/*
double SphericalManifold::getCartesianCoordinates(const tf::Vector3& normal, tf::Vector3& cartesian)
{
  cartesian = normal.normalized() * (-radius);

  tf::Vector3 origin_on_sphere = tf::Vector3(1, 0, 0);
  tf::Vector3 rotation_axis = origin_on_sphere.cross(cartesian);
  double angular_error = std::atan2(rotation_axis.length(), origin_on_sphere.dot(cartesian));

  if (fabs(angular_error) > aperture)
  {
    rotation_axis.normalize();

    cartesian = origin_on_sphere.rotate(rotation_axis, aperture);

    angular_error = fabs(angular_error) - aperture;
  }
  else
  {
    angular_error = 0;
  }

  cartesian = origin * cartesian;

  return angular_error;
}
*/
