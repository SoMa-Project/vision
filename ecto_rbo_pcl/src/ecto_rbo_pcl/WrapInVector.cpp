#include <ecto_rbo_pcl/common.h>

using namespace ecto;

namespace ecto_rbo_pcl
{

//template <typename ElementType>
//struct WrapInVector
struct WrapVector3fInVector
{
    //spore<ElementType> input_;
    //spore<std::vector<ElementType> > vector_;

    spore<UnalignedVector3f> input_;
    spore<std::vector<UnalignedVector3f> > vector_;
    
    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<UnalignedVector3f>(&WrapVector3fInVector::input_, "input", "A 4x4 affine transformation.");
        outputs.declare<std::vector<UnalignedVector3f> >(&WrapVector3fInVector::vector_, "vector", "The selected column vector.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        vector_->clear();
        vector_->push_back(*input_);
        
        return OK;
    }
};

//typedef WrapInVector<UnalignedVector4f> WrapVector4fInVector;

struct WrapAffine3fInVector
{
    //spore<ElementType> input_;
    //spore<std::vector<ElementType> > vector_;

    spore<UnalignedAffine3f> input_;
    spore<std::vector<UnalignedAffine3f> > vector_;
    
    static void declare_params(ecto::tendrils& params)
    {
    }

    static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
    {
        inputs.declare<UnalignedAffine3f>(&WrapAffine3fInVector::input_, "input", "A 4x4 affine transformation.");
        outputs.declare<std::vector<UnalignedAffine3f> >(&WrapAffine3fInVector::vector_, "vector", "An std vector with one element.");
    }

    void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
    {
    }

    int process(const tendrils& inputs, const tendrils& outputs)
    {
        vector_->clear();
        vector_->push_back(*input_);
        
        return OK;
    }
};

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::WrapVector3fInVector, "WrapVector3fInVector", "Puts an arbitrary element into an vector.");
ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::WrapAffine3fInVector, "WrapAffine3fInVector", "Puts an arbitrary element into an vector.");
