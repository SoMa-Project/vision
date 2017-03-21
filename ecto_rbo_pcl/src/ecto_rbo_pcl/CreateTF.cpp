#include <ecto_rbo_pcl/common.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

namespace ecto_rbo_pcl
{

using namespace ecto;
using ecto::pcl::PointCloud;
using ecto::pcl::Indices;
using ecto::pcl::xyz_cloud_variant_t;
using ecto::tendrils;
using ecto::spore;

struct CreateAPCTF {

    spore<UnalignedAffine3f> transform_;
    spore<std::string> bin_id_;

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        outputs.declare<UnalignedAffine3f>(&CreateAPCTF::transform_, "transform", "The reference frame of the box.");
    }

    static void declare_params(tendrils& params)
    {
        params.declare<std::string>("bin_id", "Bin of the Amazon Picking Challenge");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        bin_id_ = params["bin_id"];
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        UnalignedAffine3f transform(Eigen::Affine3f::Identity());
        Eigen::Translation3f shift_col(0.2, 0, 0);
        Eigen::Translation3f shift_row(0, 0.2, 0);

        int bins_per_row = 3;
        char alphabet[] = {'A', 'B', 'C', 'D', 'E', 'F'};
        for (int i = 0; std::string("bin_") + alphabet[i] != *bin_id_; ++i)
        {
            transform *= shift_col;

            if ((i+1) % bins_per_row == 0)
            {
                for (int j = 1; j < bins_per_row; ++j)
                    transform *= shift_col.inverse();
                transform *= shift_row;
            }
        }

        *transform_ = transform;

        return ecto::OK;
    }
};

struct CreateTF {

    spore<UnalignedAffine3f> transform_;
    spore<UnalignedVector3f> position_;
    spore<UnalignedVector4f> rotation_xyzw_;

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        outputs.declare<UnalignedAffine3f>(&CreateTF::transform_, "transform", "The reference frame of the box.");
    }

    static void declare_params(tendrils& params)
    {
        params.declare<UnalignedVector3f>("position", "Position vector.");
        params.declare<UnalignedVector4f>("rotation_xyzw", "Quaternion.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        position_ = params["position"];
        rotation_xyzw_ = params["rotation_xyzw"];
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        UnalignedAffine3f transform = Eigen::Translation3f(*position_) * Eigen::Quaternionf((*rotation_xyzw_)[3], (*rotation_xyzw_)[0], (*rotation_xyzw_)[1], (*rotation_xyzw_)[2]);
        *transform_ = transform;

        return ecto::OK;
    }
};

struct SelectTF {

    spore< std::vector<UnalignedAffine3f> > transforms_;
    spore<UnalignedAffine3f> transform_;

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare< std::vector<UnalignedAffine3f> >(&SelectTF::transforms_, "transforms", "The vector of transforms.");
        outputs.declare<UnalignedAffine3f>(&SelectTF::transform_, "transform", "The first element of the input vector.");
    }

    static void declare_params(tendrils& params)
    {
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
        *transform_ = UnalignedAffine3f::Identity();
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        if (!transforms_->empty())
        {
            UnalignedAffine3f transform = (*transforms_)[0];
            *transform_ = transform;
        }

        return ecto::OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::CreateTF, "CreateTF", "Creates a transformation. Here as a ");
ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::CreateAPCTF, "CreateAPCTF", "Creates a transformation. Here as a ");
ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::SelectTF, "SelectTF", "Selects the first element of a vector of TFs.");
