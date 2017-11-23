// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.0 (2010/01/01)

#ifndef BUMPMAPS_H
#define BUMPMAPS_H

#include "Wm5WindowApplication3.h"
#include "SimpleBumpMapEffect.h"
using namespace Wm5;

class BumpMaps : public WindowApplication3
{
    WM5_DECLARE_INITIALIZE;
    WM5_DECLARE_TERMINATE;

public:
    BumpMaps ();

    virtual bool OnInitialize ();
    virtual void OnTerminate ();
    virtual void OnIdle ();
    virtual bool OnKeyDown (unsigned char key, int x, int y);

protected:
    void CreateScene ();
    TriMesh* CreateTorus ();
    TriMesh* CreateSquare ();
    void UpdateBumpMap ();

    NodePtr mScene;
    Culler mCuller;
    AVector mLightDirection;
    bool mUseTorus;
    bool mUseBumpMap;
    Float4 mTextColor;
};

WM5_REGISTER_INITIALIZE(BumpMaps);
WM5_REGISTER_TERMINATE(BumpMaps);

#endif
