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
