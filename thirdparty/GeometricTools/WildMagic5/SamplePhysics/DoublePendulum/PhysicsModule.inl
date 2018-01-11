// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.0 (2010/01/01)

//----------------------------------------------------------------------------
inline double PhysicsModule::GetTime () const
{
    return mTime;
}
//----------------------------------------------------------------------------
inline double PhysicsModule::GetDeltaTime () const
{
    return mDeltaTime;
}
//----------------------------------------------------------------------------
inline double PhysicsModule::GetTheta1 () const
{
    return mState[0];
}
//----------------------------------------------------------------------------
inline double PhysicsModule::GetTheta1Dot () const
{
    return mState[1];
}
//----------------------------------------------------------------------------
inline double PhysicsModule::GetTheta2 () const
{
    return mState[2];
}
//----------------------------------------------------------------------------
inline double PhysicsModule::GetTheta2Dot () const
{
    return mState[3];
}
//----------------------------------------------------------------------------
