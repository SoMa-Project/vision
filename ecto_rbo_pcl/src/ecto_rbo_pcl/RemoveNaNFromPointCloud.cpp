#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <pcl/filters/filter.h>

namespace ecto_rbo_pcl {

struct RemoveNaNFromPointCloud
{
    ecto::spore<ecto::pcl::PointCloud> output_;
    ecto::spore< ::pcl::PointIndices> indices_;
    ecto::spore<bool> construct_pointcloud_;

    static void declare_params(ecto::tendrils& params)
    {
        params.declare<bool>("construct_pointcloud", "Construct the pointcloud and provid it as output or just the indices.", true);
    }

    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs) {
        outputs.declare<ecto::pcl::PointCloud>("output", "Filtered Cloud.");
        outputs.declare< ::pcl::PointIndices>("indices", "Filtered indices.");
    }

    void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
        output_ = outputs["output"];
        indices_ = outputs["indices"];
        construct_pointcloud_ = params["construct_pointcloud"];
    }

    template <typename Point>
    int process(const tendrils& inputs, const tendrils& outputs,
                boost::shared_ptr<const ::pcl::PointCloud<Point> >& input)
    {
        typename ::pcl::PointCloud<Point>::Ptr cloud(new typename ::pcl::PointCloud<Point>);
        cloud->header = input->header;
        indices_->header = input->header;
        if (*construct_pointcloud_)
            ::pcl::removeNaNFromPointCloud(*input, *cloud, indices_->indices);
//        Seems like this is not in the current PCL version? (although docs say so)
//        else
//            ::pcl::removeNaNFromPointCloud(*input, indices_->indices);

        *output_ = ecto::pcl::xyz_cloud_variant_t(cloud);

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto::pcl::PclCell<ecto_rbo_pcl::RemoveNaNFromPointCloud>, "RemoveNaNFromPointCloud", "Remove all NaNs from the point cloud.");



