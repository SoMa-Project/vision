// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.0 (2010/01/01)

#ifndef NONUNIFORMSCALE_H
#define NONUNIFORMSCALE_H

#include "Wm5WindowApplication3.h"
using namespace Wm5;

class NonuniformScale : public WindowApplication3
{
    WM5_DECLARE_INITIALIZE;
    WM5_DECLARE_TERMINATE;

public:
    NonuniformScale ();

    virtual bool OnInitialize ();
    virtual void OnTerminate ();
    virtual void OnIdle ();
    virtual bool OnKeyDown (unsigned char key, int x, int y);

protected:
    void CreateScene ();

    NodePtr mScene;
    TriMeshPtr mMesh;
    WireStatePtr mWireState;
    Culler mCuller;

    Float4 mTextColor;
};

WM5_REGISTER_INITIALIZE(NonuniformScale);
WM5_REGISTER_TERMINATE(NonuniformScale);

#endif
