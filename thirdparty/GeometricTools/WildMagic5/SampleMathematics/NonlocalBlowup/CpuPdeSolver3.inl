// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.3.0 (2010/09/07)

//----------------------------------------------------------------------------
template <typename Real>
CpuPdeSolver3<Real>::CpuPdeSolver3 (int dimension0, int dimension1,
    int dimension2, const Image3<Real>* initial,
    const Image3<unsigned char>* domain, Real dt, Real dx0, Real dx1,
    Real dx2, bool& success)
    :
    mDimension0(dimension0),
    mDimension1(dimension1),
    mDimension2(dimension2),
    mNumTexels(dimension0*dimension1*dimension2),
    mCoeff1((Real)0)
{
    // Default initialization of the remaining class members that are arrays.
    int i;
    for (i = 0; i < 4; ++i)
    {
        mCoeff0[i] = (Real)0;
    }

    // Start the construction that might have failures along the way.
    success = false;

    if (mDimension0 <= 0 || mDimension1 <= 0 || mDimension2 <= 0
    ||  dt <= (Real)0 || dx0 <= (Real)0 || dx1 <= (Real)0 || dx2 <= (Real)0)
    {
        return;
    }

    Real r0 = dt/(dx0*dx0), r1 = dt/(dx1*dx1), r2 = dt/(dx2*dx2);
    mCoeff0[0] = ((Real)1)/((Real)1 + ((Real)2)*(r0 + r1 + r2));
    mCoeff0[1] = mCoeff0[0]*r0;
    mCoeff0[2] = mCoeff0[0]*r1;
    mCoeff0[3] = mCoeff0[0]*r2;
    mCoeff1 = mCoeff0[0]*dt;

    if (initial)
    {
        for (i = 0; i < 3; ++i)
        {
            mImage[i] = *initial;
        }
    }
    else
    {
        for (i = 0; i < 3; ++i)
        {
            mImage[i].Resize(mDimension0, mDimension1, mDimension2);
            memset(mImage[i].GetPixels1D(), 0, mNumTexels*sizeof(Real));
        }
    }

    if (domain)
    {
        mMask = *domain;
    }
    else
    {
        mMask.Resize(mDimension0, mDimension1, mDimension2);
        memset(mMask.GetPixels1D(), 0xFF, mNumTexels*sizeof(unsigned char));
        int i0, i1, i2;
        for (i1 = 0; i1 < mDimension1; ++i1)
        {
            for (i0 = 0; i0 < mDimension0; ++i0)
            {
                mMask(i0, i1, 0) = 0;
                mMask(i0, i1, mDimension2 - 1) = 0;
            }
        }
        for (i2 = 0; i2 < mDimension2; ++i2)
        {
            for (i0 = 0; i0 < mDimension0; ++i0)
            {
                mMask(i0, 0, i2) = 0;
                mMask(i0, mDimension1 - 1, i2) = 0;
            }
        }
        for (i2 = 0; i2 < mDimension2; ++i2)
        {
            for (i1 = 0; i1 < mDimension1; ++i1)
            {
                mMask(0, i1, i2) = 0;
                mMask(mDimension0 - 1, i1, i2) = 0;
            }
        }
    }

    success = true;
}
//----------------------------------------------------------------------------
template <typename Real>
CpuPdeSolver3<Real>::~CpuPdeSolver3 ()
{
}
//----------------------------------------------------------------------------
template <typename Real>
bool CpuPdeSolver3<Real>::Enable ()
{
    // Stub for derived classes.
    return true;
}
//----------------------------------------------------------------------------
template <typename Real>
bool CpuPdeSolver3<Real>::Disable ()
{
    // Stub for derived classes.
    return true;
}
//----------------------------------------------------------------------------
template <typename Real>
bool CpuPdeSolver3<Real>::Execute (uint64_t iteration, int numGaussSeidel)
{
    if (iteration == 0)
    {
        mActive[0] = 0;
    }

    mActive[1] = 1 - mActive[0];
    mActive[2] = 2;

    if (!OnPreIteration(iteration))
    {
        return false;
    }

    for (int j = 0; j < numGaussSeidel; ++j)
    {
        Image3<Real>& u0 = mImage[mActive[0]];
        Image3<Real>& u1 = mImage[mActive[1]];
        Image3<Real>& u2 = mImage[mActive[2]];

        for (int i2 = 0; i2 < mDimension2; ++i2)
        {
            for (int i1 = 0; i1 < mDimension1; ++i1)
            {
                for (int i0 = 0; i0 < mDimension0; ++i0)
                {
                    if (mMask(i0,i1,i2) != 0)
                    {
                        u2(i0,i1,i2) =
                            mCoeff0[0]*u0(i0,i1,i2) +
                            mCoeff0[1]*(u1(i0+1,i1,i2) + u1(i0-1,i1,i2)) +
                            mCoeff0[2]*(u1(i0,i1+1,i2) + u1(i0,i1-1,i2)) +
                            mCoeff0[3]*(u1(i0,i1,i2+1) + u1(i0,i1,i2-1)) +
                            Equation(i0, i1, i2, u0, u1);
                    }
                    else
                    {
                        u2(i0,i1,i2) = 0.0f;
                    }
                }
            }
        }

        int save = mActive[1];
        mActive[1] = mActive[2];
        mActive[2] = save;
    }

    if (!OnPostIteration(iteration))
    {
        return false;
    }

    mActive[0] = 1 - mActive[0];
    return true;
}
//----------------------------------------------------------------------------
template <typename Real>
Real CpuPdeSolver3<Real>::Equation (int, int, int, const Image3<Real>&,
    const Image3<Real>&)
{
    // Stub for derived classes.
    return (Real)0;
}
//----------------------------------------------------------------------------
template <typename Real>
bool CpuPdeSolver3<Real>::OnPreIteration (uint64_t)
{
    // Stub for derived classes.
    return true;
}
//----------------------------------------------------------------------------
template <typename Real>
bool CpuPdeSolver3<Real>::OnPostIteration (uint64_t)
{
    // Stub for derived classes.
    return true;
}
//----------------------------------------------------------------------------
