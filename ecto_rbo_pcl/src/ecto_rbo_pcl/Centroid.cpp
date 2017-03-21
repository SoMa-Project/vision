#include <ecto_rbo_pcl/common.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

using namespace ecto;

namespace ecto_rbo_pcl
{

struct Centroid
{
  spore<std::vector< ::pcl::PointIndices> > clusters_;
  spore<std::vector<UnalignedVector4f> > centroids_;

  static void declare_params(tendrils& params)
  {
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, tendrils& outputs)
  {
    inputs.declare<std::vector< ::pcl::PointIndices> >("clusters", "Clusters indices.");
    outputs.declare<std::vector<UnalignedVector4f> >("centroids", "Clusters centroids");
  }

  void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
  {
    clusters_ = inputs["clusters"];
    centroids_ = outputs["centroids"];
  }

  template<typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
      centroids_->clear();

      // iterate over all clusters
      for (std::vector< ::pcl::PointIndices>::iterator it = clusters_->begin(); it != clusters_->end(); ++it)
      {
        // calculate mean point
        Eigen::Vector4f mean;
        ::pcl::compute3DCentroid(*input, *it, mean);
//    		  std::cout << "mean " << mean[0] << " " << mean[1] << " " << mean[2] << " " << mean[3]<< std::endl;

        centroids_->push_back(mean);
      }

      return ecto::OK;
    }
};
}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::Centroid>, "Centroid", "Calculate 3D centroids for point clusters.");
