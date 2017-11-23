// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2013/07/14)

#include "SphereMaps.h"
#include "SphereMapEffect.h"

WM5_WINDOW_APPLICATION(SphereMaps);

//----------------------------------------------------------------------------
SphereMaps::SphereMaps ()
    :
    WindowApplication3("SampleGraphics/SphereMaps", 0, 0, 640, 480,
        Float4(1.0f, 1.0f, 1.0f, 1.0f)),
        mTextColor(0.0f, 0.0f, 0.0f, 1.0f)
{
    Environment::InsertDirectory(ThePath + "Shaders/");
}
//----------------------------------------------------------------------------
bool SphereMaps::OnInitialize ()
{
    if (!WindowApplication3::OnInitialize())
    {
        return false;
    }

    CreateScene();

    // Center-and-fit for camera viewing.
    mScene->Update();
    mTrnNode->LocalTransform.SetTranslate(-mScene->WorldBound.GetCenter());
    mCamera->SetFrustum(60.0f, GetAspectRatio(), 1.0f, 1000.0f);
    AVector camDVector(0.0f, 1.0f, 0.0f);
    AVector camUVector(0.0f, 0.0f, 1.0f);
    AVector camRVector = camDVector.Cross(camUVector);
    APoint camPosition = APoint::ORIGIN -
        3.0f*mScene->WorldBound.GetRadius()*camDVector;
    mCamera->SetFrame(camPosition, camDVector, camUVector, camRVector);

    // Initial update of objects.
    mScene->Update();
    CopyNormalToTCoord1(mScene);

    // Initial culling of scene.
    mCuller.SetCamera(mCamera);
    mCuller.ComputeVisibleSet(mScene);

    InitializeCameraMotion(0.001f, 0.001f);
    InitializeObjectMotion(mScene);
    return true;
}
//----------------------------------------------------------------------------
void SphereMaps::OnTerminate ()
{
    mScene = 0;
    mTrnNode = 0;

    WindowApplication3::OnTerminate();
}
//----------------------------------------------------------------------------
void SphereMaps::OnIdle ()
{
    MeasureTime();

    if (MoveCamera())
    {
        mCuller.ComputeVisibleSet(mScene);
    }

    if (MoveObject())
    {
        mScene->Update();
        CopyNormalToTCoord1(mScene);
        mCuller.ComputeVisibleSet(mScene);
    }

    if (mRenderer->PreDraw())
    {
        mRenderer->ClearBuffers();
        mRenderer->Draw(mCuller.GetVisibleSet());
        DrawFrameRate(8, GetHeight()-8, mTextColor);
        mRenderer->PostDraw();
        mRenderer->DisplayColorBuffer();
    }

    UpdateFrameCount();
}
//----------------------------------------------------------------------------
void SphereMaps::CreateScene ()
{
    mScene = new0 Node();
    mTrnNode = new0 Node();
    mScene->AttachChild(mTrnNode);

    VertexFormat* vformat = VertexFormat::Create(3,
        VertexFormat::AU_POSITION, VertexFormat::AT_FLOAT3, 0,
        VertexFormat::AU_NORMAL, VertexFormat::AT_FLOAT3, 0,
        VertexFormat::AU_TEXCOORD, VertexFormat::AT_FLOAT3, 1);

    TriMesh* mesh = StandardMesh(vformat).Torus(64, 64, 1.0f, 0.5f);
    mTrnNode->AttachChild(mesh);

    std::string effectFile = Environment::GetPathR("SphereMap.wmfx");
    SphereMapEffect* effect = new0 SphereMapEffect(effectFile);
    std::string environmentName = Environment::GetPathR("SphereMap.wmtf");
    Texture2D* environmentTexture = Texture2D::LoadWMTF(environmentName);
    environmentTexture->GenerateMipmaps();
    mesh->SetEffectInstance(effect->CreateInstance(environmentTexture));
}
//----------------------------------------------------------------------------
void SphereMaps::CopyNormalToTCoord1 (Object* object)
{
    TriMesh* mesh = DynamicCast<TriMesh>(object);
    if (mesh)
    {
        VertexBufferAccessor vba(mesh);
        for (int i = 0; i < vba.GetNumVertices(); ++i)
        {
            vba.TCoord<Vector3f>(1, i) = vba.Normal<Vector3f>(i);
        }
        mRenderer->Update(mesh->GetVertexBuffer());
    }

    Node* node = DynamicCast<Node>(object);
    if (node)
    {
        for (int i = 0; i < node->GetNumChildren(); ++i)
        {
            Spatial* child = node->GetChild(i);
            if (child)
            {
                CopyNormalToTCoord1(child);
            }
        }
    }
}
//----------------------------------------------------------------------------
