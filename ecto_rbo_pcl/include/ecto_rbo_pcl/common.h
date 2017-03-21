/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <Eigen/Core>

#include <ecto/ecto.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>
#include <ecto_pcl/pcl_cell_dual_inputs.hpp>

namespace ecto_rbo_pcl
{
  typedef std::vector<std::vector<int> > ClustersAsStdInt;
  
  // defining unaligned versions of eigen vectors and matrices
  typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
  typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
  typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
  typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> UnalignedMatrix3f;
  typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;
}

#endif /* COMMON_H_ */
