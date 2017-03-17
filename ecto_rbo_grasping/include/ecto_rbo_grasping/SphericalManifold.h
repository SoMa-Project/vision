/*
 * SphericalManifold.h
 *
 *  Created on: Mar 4, 2013
 *      Author: clemens
 */

#ifndef SPHERICALMANIFOLD_H_
#define SPHERICALMANIFOLD_H_

#include <Eigen/Geometry>

class SphericalManifold
{
public:
  SphericalManifold(double radius, double aperture);
  virtual ~SphericalManifold();

  double getRadius() const;
  double getAperture() const;

  void setOrigin(Eigen::Vector3f& translation, Eigen::Vector3f& y_axis, Eigen::Vector3f& x_axis);

  // return the coordinate that is optimal with respect to the orientation error
  double getCartesianCoordinate(double inclination, double azimuth, Eigen::Vector3f& cartesian);

  // return the coordinates that are optimal with respect to the orientaion error (there are MULTIPLE coordinates with the same orientation/normal !)
  double getCartesianCoordinate(Eigen::Vector3f& desired_normal, Eigen::Vector3f& resulting_normal, Eigen::Vector3f& cartesian);

  // return the coordinate that is optimal with respect to position along one dimension
  //void getCartesianCoordinateDim(const Eigen::Vector3f& positive_dimension, Eigen::Vector3f& cartesian);

private:
  Eigen::Transform<float, 3, Eigen::Affine> origin;

  double radius;

  double aperture;
};

#endif /* SPHERICALMANIFOLD_H_ */
