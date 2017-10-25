// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.3.0 (2010/09/07)

#ifndef GPUNONLOCALSOLVER3_H
#define GPUNONLOCALSOLVER3_H

#define DO_ALL_GPU

#include "GpuPdeSolver3.h"

#ifdef DO_ALL_GPU
#include "GpuMaxPyramid2.h"
#include "GpuEvaluate2.h"
#include "GpuAvrPyramid2.h"
#endif

class GpuNonlocalSolver3 : public GpuPdeSolver3
{
public:
    GpuNonlocalSolver3 (int dimension0, int dimension1, int dimension2,
        const Image3<float>* initial, const Image3<unsigned char>* domain,
        float dt, float dx0, float dx1, float dx2, float p,
        const std::string& folder, bool& success);

    virtual ~GpuNonlocalSolver3 ();

private:
#ifdef DO_ALL_GPU
    class Evaluator : public GpuEvaluate2
    {
    public:
        Evaluator (int bound0, int bound1, bool& success);
        virtual ~Evaluator ();

        void SetUMax (float umax);

    protected:
        virtual bool OnPreEvaluation (GLuint texture, GLuint frameBuffer);

        GLint mUMaxLocation;
        float mUMax;

        static const GLchar* msDeclarations;
        static const GLchar* msEquation;
    };
#else
    void GetIntegral (float& umax, float& integral);
#endif

    virtual bool OnPreIteration (uint64_t iteration);
    virtual bool OnPostIteration (uint64_t iteration);

    float mPower;
    float* mSlice;
    float mNonlinear0, mNonlinear1;
    GLint mNonlinearLocation;
    std::string mFolder;

#ifdef DO_ALL_GPU
    GpuMaxPyramid2 mMaxPyramid;
    Evaluator mEvaluator;
    GpuAvrPyramid2 mAvrPyramid;
#else
    float* mReadBack;
#endif

    static const GLchar* msDeclarations;
    static const GLchar* msEquation;
};

#endif
