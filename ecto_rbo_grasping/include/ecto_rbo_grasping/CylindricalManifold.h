/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

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
