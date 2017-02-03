/*
Copyright 2016-2017 Robotics and Biology Lab, TU Berlin. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted as representing official policies, either expressed or implied, of the FreeBSD Project.
*/ 

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>


using namespace ecto;

namespace ecto_rbo_pcl
{

typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> UnalignedVector4f;
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> UnalignedVector3f;
typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> UnalignedVector2f;
typedef Eigen::Transform<float,3,Eigen::Affine,Eigen::DontAlign> UnalignedAffine3f;

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

}

ECTO_CELL(ecto_rbo_pcl, ecto_rbo_pcl::WrapVector3fInVector, "WrapVector3fInVector", "Puts an arbitrary element into an vector.");
