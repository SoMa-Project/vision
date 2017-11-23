// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.0 (2010/01/01)

#ifndef PHYSICSMODULE_H
#define PHYSICSMODULE_H

class PhysicsModule
{
public:
    // Construction and destruction.
    PhysicsModule ();
    ~PhysicsModule ();

    // Initialize the differential equation solver.
    void Initialize (double time, double deltaTime, double x, double y,
        double theta, double xDer, double yDer, double thetaDer);

    // Access the current state.
    inline double GetTime () const;
    inline double GetDeltaTime () const;
    inline double GetX () const;
    inline double GetXDer () const;
    inline double GetY () const;
    inline double GetYDer () const;
    inline double GetTheta () const;
    inline double GetThetaDer () const;
    void Get (double& x1, double& y1, double& x2, double& y2) const;

    // Apply a single step of the solver.
    void Update ();

    // physical constants   // symbols used in the Game Physics book
    double MuGravity;       // mu*g
    double Length;          // L1 = L2 = L/2

protected:
    // state and auxiliary variables
    double mTime, mDeltaTime;
    double mX, mY, mTheta, mXDer, mYDer, mThetaDer;
    double mX0, mY0, mTheta0, mXDer0, mYDer0, mThetaDer0;
    double mHalfLength, mLinVelCoeff, mAngVelCoeff;
};

#include "PhysicsModule.inl"

#endif
