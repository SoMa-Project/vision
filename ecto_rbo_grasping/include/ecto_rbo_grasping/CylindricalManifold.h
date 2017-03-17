/*
 * CylindricalManifold.h
 *
 *  Created on: Mar 4, 2013
 *      Author: clemens
 */

#ifndef CYLINDRICALMANIFOLD_H_
#define CYLINDRICALMANIFOLD_H_

#include <Eigen/Geometry>

class CylindricalManifold
{
public:
  CylindricalManifold(double radius, double height, double aperture);
  virtual ~CylindricalManifold();

  double getRadius() const;
  double getHeight() const;
  double getAperture() const;

  void setOrigin(Eigen::Vector3f& translation, Eigen::Vector3f& cylindrical_axis, Eigen::Vector3f& x_axis);

  double getCartesianCoordinate(double phi, double z, Eigen::Vector3f& cartesian);

  // return the coordinates that are optimal with respect to the orientaion error (there are MULTIPLE coordinates with the same orientation/normal !)
  double getCartesianCoordinate(Eigen::Vector3f& desired_normal, Eigen::Vector3f& resulting_normal, Eigen::Vector3f& cartesian);

  // return the coordinate that is optimal with respect to position along one dimension
  void getCartesianCoordinateDim(const Eigen::Vector3f& positive_dimension, Eigen::Vector3f& cartesian);

private:
//  tf::Transform origin;
  Eigen::Transform<float, 3, Eigen::Affine> origin;

  // euclidean distance between cylindrical axis (origin(:,2)) and 2D plane
  double radius;

  // interval for z-values (== height)
  double height;

  // interval for azimuth-values
  double aperture;

};

#endif /* CYLINDRICALMANIFOLD_H_ */
