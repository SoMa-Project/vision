// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.0 (2010/01/01)

//----------------------------------------------------------------------------
inline int FloatArray::GetNumElements () const
{
    return mNumElements;
}
//----------------------------------------------------------------------------
inline float* FloatArray::GetData () const
{
    return mElements;
}
//----------------------------------------------------------------------------
inline FloatArray::operator const float* () const
{
    return mElements;
}
//----------------------------------------------------------------------------
inline FloatArray::operator float* ()
{
    return mElements;
}
//----------------------------------------------------------------------------
inline const float& FloatArray::operator[] (int i) const
{
    return mElements[i];
}
//----------------------------------------------------------------------------
inline float& FloatArray::operator[] (int i)
{
    return mElements[i];
}
//----------------------------------------------------------------------------
