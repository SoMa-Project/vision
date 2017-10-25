// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2013/07/14)

#include "Wm5GraphicsPCH.h"
#include "Wm5LightSptPerPixEffect.h"
#include "Wm5CameraModelPositionConstant.h"
#include "Wm5MaterialEmissiveConstant.h"
#include "Wm5MaterialAmbientConstant.h"
#include "Wm5MaterialDiffuseConstant.h"
#include "Wm5MaterialSpecularConstant.h"
#include "Wm5LightAmbientConstant.h"
#include "Wm5LightDiffuseConstant.h"
#include "Wm5LightSpecularConstant.h"
#include "Wm5LightAttenuationConstant.h"
#include "Wm5LightModelDVectorConstant.h"
#include "Wm5LightModelPositionConstant.h"
#include "Wm5LightSpotConstant.h"
#include "Wm5PVWMatrixConstant.h"
#include "Wm5WMatrixConstant.h"
using namespace Wm5;

WM5_IMPLEMENT_RTTI(Wm5, VisualEffect, LightSptPerPixEffect);
WM5_IMPLEMENT_STREAM(LightSptPerPixEffect);
WM5_IMPLEMENT_FACTORY(LightSptPerPixEffect);
WM5_IMPLEMENT_DEFAULT_NAMES(VisualEffect, LightSptPerPixEffect);

//----------------------------------------------------------------------------
LightSptPerPixEffect::LightSptPerPixEffect ()
{
    VertexShader* vshader = new0 VertexShader("Wm5.LightSptPerPix",
        2, 3, 1, 0, false);
    vshader->SetInput(0, "modelPosition", Shader::VT_FLOAT3,
        Shader::VS_POSITION);
    vshader->SetInput(1, "modelNormal", Shader::VT_FLOAT3,
        Shader::VS_TEXCOORD1);
    vshader->SetOutput(0, "clipPosition", Shader::VT_FLOAT4,
        Shader::VS_POSITION);
    vshader->SetOutput(1, "vertexPosition", Shader::VT_FLOAT3,
        Shader::VS_TEXCOORD0);
    vshader->SetOutput(2, "vertexNormal", Shader::VT_FLOAT3,
        Shader::VS_TEXCOORD1);
    vshader->SetConstant(0, "PVWMatrix", 4);
    vshader->SetBaseRegisters(msVRegisters);
    vshader->SetPrograms(msVPrograms);

    PixelShader* pshader = new0 PixelShader("Wm5.LightSptPerPix",
        2, 1, 13, 0, false);
    pshader->SetInput(0, "vertexPosition", Shader::VT_FLOAT3,
        Shader::VS_TEXCOORD0);
    pshader->SetInput(1, "vertexNormal", Shader::VT_FLOAT3,
        Shader::VS_TEXCOORD1);
    pshader->SetOutput(0, "pixelColor", Shader::VT_FLOAT4,
        Shader::VS_COLOR0);
    pshader->SetConstant(0, "WMatrix", 4);
    pshader->SetConstant(1, "CameraModelPosition", 1);
    pshader->SetConstant(2, "MaterialEmissive", 1);
    pshader->SetConstant(3, "MaterialAmbient", 1);
    pshader->SetConstant(4, "MaterialDiffuse", 1);
    pshader->SetConstant(5, "MaterialSpecular", 1);
    pshader->SetConstant(6, "LightModelPosition", 1);
    pshader->SetConstant(7, "LightModelDirection", 1);
    pshader->SetConstant(8, "LightAmbient", 1);
    pshader->SetConstant(9, "LightDiffuse", 1);
    pshader->SetConstant(10, "LightSpecular", 1);
    pshader->SetConstant(11, "LightSpotCutoff", 1);
    pshader->SetConstant(12, "LightAttenuation", 1);
    pshader->SetBaseRegisters(msPRegisters);
    pshader->SetPrograms(msPPrograms);

    VisualPass* pass = new0 VisualPass();
    pass->SetVertexShader(vshader);
    pass->SetPixelShader(pshader);
    pass->SetAlphaState(new0 AlphaState());
    pass->SetCullState(new0 CullState());
    pass->SetDepthState(new0 DepthState());
    pass->SetOffsetState(new0 OffsetState());
    pass->SetStencilState(new0 StencilState());
    pass->SetWireState(new0 WireState());

    VisualTechnique* technique = new0 VisualTechnique();
    technique->InsertPass(pass);
    InsertTechnique(technique);
}
//----------------------------------------------------------------------------
LightSptPerPixEffect::~LightSptPerPixEffect ()
{
}
//----------------------------------------------------------------------------
VisualEffectInstance* LightSptPerPixEffect::CreateInstance (Light* light,
    Material* material) const
{
    VisualEffectInstance* instance = new0 VisualEffectInstance(this, 0);
    instance->SetVertexConstant(0, 0,
        new0 PVWMatrixConstant());
    instance->SetPixelConstant(0, 0,
        new0 WMatrixConstant());
    instance->SetPixelConstant(0, 1,
        new0 CameraModelPositionConstant());
    instance->SetPixelConstant(0, 2,
        new0 MaterialEmissiveConstant(material));
    instance->SetPixelConstant(0, 3,
        new0 MaterialAmbientConstant(material));
    instance->SetPixelConstant(0, 4,
        new0 MaterialDiffuseConstant(material));
    instance->SetPixelConstant(0, 5,
        new0 MaterialSpecularConstant(material));
    instance->SetPixelConstant(0, 6,
        new0 LightModelPositionConstant(light));
    instance->SetPixelConstant(0, 7,
        new0 LightModelDVectorConstant(light));
    instance->SetPixelConstant(0, 8,
        new0 LightAmbientConstant(light));
    instance->SetPixelConstant(0, 9,
        new0 LightDiffuseConstant(light));
    instance->SetPixelConstant(0, 10,
        new0 LightSpecularConstant(light));
    instance->SetPixelConstant(0, 11,
        new0 LightSpotConstant(light));
    instance->SetPixelConstant(0, 12,
        new0 LightAttenuationConstant(light));
    return instance;
}
//----------------------------------------------------------------------------
VisualEffectInstance* LightSptPerPixEffect::CreateUniqueInstance (
    Light* light, Material* material)
{
    LightSptPerPixEffect* effect = new0 LightSptPerPixEffect();
    return effect->CreateInstance(light, material);
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Streaming support.
//----------------------------------------------------------------------------
LightSptPerPixEffect::LightSptPerPixEffect (LoadConstructor value)
    :
    VisualEffect(value)
{
}
//----------------------------------------------------------------------------
void LightSptPerPixEffect::Load (InStream& source)
{
    WM5_BEGIN_DEBUG_STREAM_LOAD(source);

    VisualEffect::Load(source);

    WM5_END_DEBUG_STREAM_LOAD(LightSptPerPixEffect, source);
}
//----------------------------------------------------------------------------
void LightSptPerPixEffect::Link (InStream& source)
{
    VisualEffect::Link(source);
}
//----------------------------------------------------------------------------
void LightSptPerPixEffect::PostLink ()
{
    VisualEffect::PostLink();

    VisualPass* pass = mTechniques[0]->GetPass(0);
    VertexShader* vshader = pass->GetVertexShader();
    PixelShader* pshader = pass->GetPixelShader();
    vshader->SetBaseRegisters(msVRegisters);
    vshader->SetPrograms(msVPrograms);
    pshader->SetBaseRegisters(msVRegisters);
    pshader->SetPrograms(msPPrograms);
}
//----------------------------------------------------------------------------
bool LightSptPerPixEffect::Register (OutStream& target) const
{
    return VisualEffect::Register(target);
}
//----------------------------------------------------------------------------
void LightSptPerPixEffect::Save (OutStream& target) const
{
    WM5_BEGIN_DEBUG_STREAM_SAVE(target);

    VisualEffect::Save(target);

    WM5_END_DEBUG_STREAM_SAVE(LightSptPerPixEffect, target);
}
//----------------------------------------------------------------------------
int LightSptPerPixEffect::GetStreamingSize () const
{
    return VisualEffect::GetStreamingSize();
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Profiles.
//----------------------------------------------------------------------------
int LightSptPerPixEffect::msDx9VRegisters[1] = { 0 };
int LightSptPerPixEffect::msOglVRegisters[1] = { 1 };
int* LightSptPerPixEffect::msVRegisters[Shader::MAX_PROFILES] =
{
    0,
    msDx9VRegisters,
    msDx9VRegisters,
    msDx9VRegisters,
    msOglVRegisters
};

std::string LightSptPerPixEffect::msVPrograms[Shader::MAX_PROFILES] =
{
    // VP_NONE
    "",

    // VP_VS_1_1
    "",

    // VP_VS_2_0
    "vs_2_0\n"
    "def c4, 1.00000000, 0, 0, 0\n"
    "dcl_position v0\n"
    "dcl_texcoord1 v1\n"
    "mov r0.w, c4.x\n"
    "mov r0.xyz, v0\n"
    "dp4 oPos.w, r0, c3\n"
    "dp4 oPos.z, r0, c2\n"
    "dp4 oPos.y, r0, c1\n"
    "dp4 oPos.x, r0, c0\n"
    "mov oT0.xyz, v0\n"
    "mov oT1.xyz, v1\n",

    // VP_VS_3_0
    "vs_3_0\n"
    "dcl_position o0\n"
    "dcl_texcoord0 o1\n"
    "dcl_texcoord1 o2\n"
    "def c4, 1.00000000, 0, 0, 0\n"
    "dcl_position0 v0\n"
    "dcl_texcoord1 v1\n"
    "mov r0.w, c4.x\n"
    "mov r0.xyz, v0\n"
    "dp4 o0.w, r0, c3\n"
    "dp4 o0.z, r0, c2\n"
    "dp4 o0.y, r0, c1\n"
    "dp4 o0.x, r0, c0\n"
    "mov o1.xyz, v0\n"
    "mov o2.xyz, v1\n",

    // VP_ARBVP1
    "!!ARBvp1.0\n"
    "PARAM c[5] = { { 1 }, program.local[1..4] };\n"
    "TEMP R0;\n"
    "MOV R0.w, c[0].x;\n"
    "MOV R0.xyz, vertex.position;\n"
    "DP4 result.position.w, R0, c[4];\n"
    "DP4 result.position.z, R0, c[3];\n"
    "DP4 result.position.y, R0, c[2];\n"
    "DP4 result.position.x, R0, c[1];\n"
    "MOV result.texcoord[0].xyz, vertex.position;\n"
    "MOV result.texcoord[1].xyz, vertex.texcoord[1];\n"
    "END\n"
};

int LightSptPerPixEffect::msAllPRegisters[13] =
    { 0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

int* LightSptPerPixEffect::msPRegisters[Shader::MAX_PROFILES] =
{
    0,
    msAllPRegisters,
    msAllPRegisters,
    msAllPRegisters,
    msAllPRegisters
};

std::string LightSptPerPixEffect::msPPrograms[Shader::MAX_PROFILES] =
{
    // PP_NONE
    "",

    // PP_PS_1_1
    "",

    // PP_PS_2_0
    "ps_2_0\n"
    "def c14, 0.00000000, 0, 0, 0\n"
    "dcl t1.xyz\n"
    "dcl t0.xyz\n"
    "add r2.xyz, t0, -c9\n"
    "dp3 r0.x, r2, r2\n"
    "rsq r3.x, r0.x\n"
    "add r1.xyz, -t0, c4\n"
    "dp3 r0.x, r1, r1\n"
    "mul r3.xyz, r3.x, r2\n"
    "rsq r0.x, r0.x\n"
    "mad r4.xyz, r0.x, r1, -r3\n"
    "dp3 r1.x, r4, r4\n"
    "dp3 r0.x, t1, t1\n"
    "rsq r1.x, r1.x\n"
    "mul r1.xyz, r1.x, r4\n"
    "rsq r0.x, r0.x\n"
    "mul r4.xyz, r0.x, t1\n"
    "dp3 r0.x, r4, r1\n"
    "max r0.x, r0, c14\n"
    "pow r1, r0.x, c8.w\n"
    "dp3 r0.x, r4, r3\n"
    "dp3 r3.z, r2, c2\n"
    "dp3 r3.x, r2, c0\n"
    "dp3 r3.y, r2, c1\n"
    "dp3 r2.x, r3, r3\n"
    "cmp r3.x, r0, c14, r1\n"
    "rsq r1.x, r2.x\n"
    "mul r2.xyz, r3.x, c8\n"
    "max r3.x, -r0, c14\n"
    "rcp r1.x, r1.x\n"
    "mad r0.x, r1, c13.z, c13.y\n"
    "mad r0.x, r1, r0, c13\n"
    "mul r3.xyz, r3.x, c7\n"
    "rcp r0.x, r0.x\n"
    "mul r3.xyz, r3, c11\n"
    "mov r1.xyz, c6\n"
    "mad r1.xyz, c10, r1, r3\n"
    "mad r1.xyz, r2, c12, r1\n"
    "mul r0.x, r0, c13.w\n"
    "mad r0.xyz, r0.x, r1, c5\n"
    "mov r0.w, c7\n"
    "mov oC0, r0\n",

    // PP_PS_3_0
    "ps_3_0\n"
    "def c16, 1.00000000, 0.00000000, 0, 0\n"
    "dcl_texcoord1 v1.xyz\n"
    "dcl_texcoord0 v0.xyz\n"
    "add r1.xyz, v0, -c9\n"
    "dp3 r0.w, r1, r1\n"
    "add r0.xyz, -v0, c4\n"
    "rsq r0.w, r0.w\n"
    "mul r2.xyz, r0.w, r1\n"
    "dp3 r1.w, r0, r0\n"
    "rsq r0.w, r1.w\n"
    "mad r0.xyz, r0.w, r0, -r2\n"
    "dp3 r1.w, r0, r0\n"
    "rsq r1.w, r1.w\n"
    "dp3 r0.w, v1, v1\n"
    "rsq r0.w, r0.w\n"
    "mul r3.xyz, r0.w, v1\n"
    "mul r0.xyz, r1.w, r0\n"
    "dp3 r0.x, r3, r0\n"
    "max r1.w, r0.x, c16.y\n"
    "pow r0, r1.w, c8.w\n"
    "mov r0.y, r0.x\n"
    "dp3 r0.x, r2, r3\n"
    "dp3 r1.w, r2, c10\n"
    "dp3 r2.z, r1, c2\n"
    "dp3 r2.y, r1, c1\n"
    "cmp r3.y, r0.x, c16, r0\n"
    "max r3.x, -r0, c16.y\n"
    "pow r0, r1.w, c14.w\n"
    "add r2.x, r1.w, -c14.y\n"
    "cmp r0.x, r2, c16, c16.y\n"
    "dp3 r2.x, r1, c0\n"
    "mov r3.z, r0.w\n"
    "abs_pp r0.x, r0\n"
    "cmp r0.xyz, -r0.x, c16.y, r3\n"
    "dp3 r0.w, r2, r2\n"
    "mul r1.xyz, r0.x, c7\n"
    "mul r2.xyz, r0.y, c8\n"
    "mul r2.xyz, r2, c13\n"
    "mad r1.xyz, r1, c12, r2\n"
    "mul r2.xyz, r0.z, r1\n"
    "rsq r0.x, r0.w\n"
    "rcp r0.x, r0.x\n"
    "mad r0.y, r0.x, c15.z, c15\n"
    "mad r0.x, r0, r0.y, c15\n"
    "rcp r0.x, r0.x\n"
    "mov r1.xyz, c6\n"
    "mad r1.xyz, c11, r1, r2\n"
    "mul r0.x, r0, c15.w\n"
    "mad oC0.xyz, r0.x, r1, c5\n"
    "mov oC0.w, c7\n",

    // PP_ARBFP1
    "!!ARBfp1.0\n"
    "PARAM c[17] = { program.local[0..15], { 0, 1 } };\n"
    "TEMP R0;\n"
    "TEMP R1;\n"
    "TEMP R2;\n"
    "TEMP R3;\n"
    "ADD R0.xyz, fragment.texcoord[0], -c[9];\n"
    "DP3 R0.w, R0, R0;\n"
    "ADD R2.xyz, -fragment.texcoord[0], c[4];\n"
    "RSQ R0.w, R0.w;\n"
    "MUL R1.xyz, R0.w, R0;\n"
    "DP3 R1.w, R2, R2;\n"
    "RSQ R0.w, R1.w;\n"
    "MAD R2.xyz, R0.w, R2, -R1;\n"
    "DP3 R1.w, R2, R2;\n"
    "RSQ R1.w, R1.w;\n"
    "DP3 R0.w, fragment.texcoord[1], fragment.texcoord[1];\n"
    "MUL R3.xyz, R1.w, R2;\n"
    "RSQ R0.w, R0.w;\n"
    "MUL R2.xyz, R0.w, fragment.texcoord[1];\n"
    "DP3 R3.y, R2, R3;\n"
    "DP3 R1.w, R1, R2;\n"
    "DP3 R0.w, R1, c[10];\n"
    "MOV R3.x, -R1.w;\n"
    "SGE R1.x, R0.w, c[14].y;\n"
    "MOV R3.z, c[8].w;\n"
    "ABS R1.x, R1;\n"
    "DP3 R2.z, R0, c[2];\n"
    "DP3 R2.x, R0, c[0];\n"
    "DP3 R2.y, R0, c[1];\n"
    "POW R1.w, R0.w, c[14].w;\n"
    "DP3 R0.w, R2, R2;\n"
    "RSQ R0.w, R0.w;\n"
    "RCP R0.w, R0.w;\n"
    "LIT R1.yz, R3.xyzz;\n"
    "CMP R1.x, -R1, c[16], c[16].y;\n"
    "CMP R1.xyz, -R1.x, c[16].x, R1.yzww;\n"
    "MUL R0.xyz, R1.x, c[7];\n"
    "MAD R1.x, R0.w, c[15].z, c[15].y;\n"
    "MAD R0.w, R0, R1.x, c[15].x;\n"
    "MUL R2.xyz, R1.y, c[8];\n"
    "MUL R2.xyz, R2, c[13];\n"
    "MAD R0.xyz, R0, c[12], R2;\n"
    "MUL R0.xyz, R1.z, R0;\n"
    "MOV R1.xyz, c[11];\n"
    "MAD R1.xyz, R1, c[6], R0;\n"
    "RCP R0.w, R0.w;\n"
    "MUL R0.x, R0.w, c[15].w;\n"
    "MAD result.color.xyz, R0.x, R1, c[5];\n"
    "MOV result.color.w, c[7];\n"
    "END\n"
};
//----------------------------------------------------------------------------
