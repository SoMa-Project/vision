// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.0 (2010/01/01)

#ifndef SIMPLEPENDULUMFRICTION_H
#define SIMPLEPENDULUMFRICTION_H

#include "Wm5WindowApplication3.h"
#include "PhysicsModule.h"
using namespace Wm5;

class SimplePendulumFriction : public WindowApplication3
{
    WM5_DECLARE_INITIALIZE;
    WM5_DECLARE_TERMINATE;

public:
    SimplePendulumFriction ();

    virtual bool OnInitialize ();
    virtual void OnTerminate ();
    virtual void OnIdle ();
    virtual bool OnKeyDown (unsigned char key, int x, int y);

protected:
    void InitializeModule ();
    void CreateScene ();
    TriMesh* CreateFloor ();
    Polypoint* CreatePath ();
    Node* CreatePendulum ();
    void PhysicsTick ();
    void GraphicsTick ();

    // The scene graph.
    NodePtr mScene, mPendulum;
    WireStatePtr mWireState;
    Culler mCuller;

    // The physics system.
    PhysicsModule mModule;
    int mMotionType;

    // Controlled frame rate.
    float mLastIdle;

    Float4 mTextColor;
};

WM5_REGISTER_INITIALIZE(SimplePendulumFriction);
WM5_REGISTER_TERMINATE(SimplePendulumFriction);

#endif
