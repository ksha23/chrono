// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: (Metal RT backend)
// =============================================================================
// Metal Shading Language kernel for Chrono::Sensor Metal RT cameras.
// Inline ray query (hardware ray tracing). Modes: 0 color, 1 depth, 2 normal,
// 3 segmentation. Output is RGBA32Float; the filter converts to the sensor's
// host buffer format.
// =============================================================================

#ifndef CH_METAL_RT_SHADER_MSL_H
#define CH_METAL_RT_SHADER_MSL_H

static const char* kRaytraceMSL = R"MSLGEN(
#include <metal_stdlib>
#include <metal_raytracing>
using namespace metal;
using namespace raytracing;

struct Uniforms {
    float3 camPos; float3 camRight; float3 camUp; float3 camForward; float3 ambient;
    float tanHalfFov; uint width; uint height; uint aa; uint numLights; uint mode;
    float lidarHFov; float lidarVMin; float lidarVMax; float maxDist;
    uint lidarSampleRadius; float lidarHDiv; float lidarVDiv; uint lidarReturnMode;
    uint lensModel; float dk1; float dk2; float dk3;  // 0 pinhole, 1 FOV(fisheye), 2 radial
    uint hasEnv;                                       // 1 = sample envTex (HDR equirect) for sky/reflections
    uint bgMode;                                       // background when no env: 0 procedural, 1 gradient, 2 solid
    packed_float3 bgZenith; packed_float3 bgHorizon;   // gradient (zenith/horizon) or solid (zenith)
    packed_float3 fogColor; float fogScatter;          // exponential fog (0 scatter = off)
    uint useGi;                                        // 1 = path-traced global illumination
    float exposure;                                    // linear exposure/gain (1 = none)
    float vignette;                                    // vignette strength (0 = none)
    float apertureR;                                   // lens aperture radius for depth of field (0 = pinhole)
    float focalDist;                                   // DoF focal distance
    float noiseSigma;                                  // gaussian sensor noise stddev (0 = none)
    float envIntensity;                                // environment-map radiance scale (OptiX AddEnvironmentLight intensity_scale)
    float gamma;                                       // output gamma (OptiX camera.gamma; 2.2 = sRGB, 1 = linear)
};

// Camera ray direction for a normalized image coord (nx includes aspect, ny does not).
static inline float3 camRayDir(float nx, float ny, constant Uniforms& u) {
    float3 right=float3(u.camRight), up=float3(u.camUp), fwd=float3(u.camForward);
    if (u.lensModel==1u) {                                  // FOV_LENS: equidistant fisheye
        float r=sqrt(nx*nx+ny*ny); float theta=r*(u.lidarHFov*0.5);
        if (r<1e-6) return normalize(fwd);
        float3 axis=(nx*right+ny*up)/r;
        return normalize(cos(theta)*fwd + sin(theta)*axis);
    }
    float k=1.0;
    if (u.lensModel==2u) { float r2=nx*nx+ny*ny; k=1.0+u.dk1*r2+u.dk2*r2*r2+u.dk3*r2*r2*r2; }  // RADIAL
    float px=nx*k*u.tanHalfFov, py=ny*k*u.tanHalfFov;
    return normalize(px*right+py*up+fwd);
}
// type: 0 point, 1 directional, 2 spot. dir/cosOuter/cosInner used by spot only.
struct Light { packed_float3 pos; float range; packed_float3 color; float type; packed_float3 dir; float cosOuter; float cosInner; float p0; float p1; float p2; };

static inline float3 skycol(float3 d, texture2d<float> env, sampler samp, constant Uniforms& u){
  float3 dn=normalize(d);
  if(u.hasEnv!=0u){                                      // HDR equirectangular environment map (matches OptiX miss.cu)
    float uu=atan2(dn.y,dn.x)*(0.5/M_PI_F)+0.5;
    float v=acos(clamp(dn.z,-1.0,1.0))*(1.0/M_PI_F);     // z-up: zenith -> v=0 (top of image)
    // .hdr is linear radiance -> sample linearly and scale by intensity, matching OptiX's environment LIGHT
    // (ChOptixEnvironmentLight: L = tex.rgb * intensity_scale). NB: OptiX's LEGACY background (miss.cu) does a
    // pow(2.2) instead, but scenes that use the env for lighting (e.g. teleopcity, GI) read the map linearly
    // and brighter -- that is what the driving demos compare against, so we match the light path here.
    return max(env.sample(samp,float2(uu,v)).rgb,0.0) * u.envIntensity;
  }
  if(u.bgMode==1u){ float m=max(0.0,dn.z); return m*float3(u.bgZenith)+(1.0-m)*float3(u.bgHorizon); }  // GRADIENT (OptiX miss.cu)
  return float3(u.bgZenith);                                                                            // SOLID (OptiX default = black)
}

// Microfacet BRDF terms ported verbatim from Chrono's OptiX shader_utils.cuh (GGX D, Hammon-Smith G;
// the 1/pi in D and the 4*NdV*NdL in G are omitted exactly as in the OptiX code).
static inline float NormalDist(float NdH, float rough){ float r2=rough*rough; float d=NdH*NdH*(r2-1.0)+1.0; return r2/max(d*d,1e-6); }
static inline float HammonSmith(float NdV, float NdL, float rough){ float den=mix(2.0*abs(NdV)*abs(NdL), abs(NdL)+abs(NdV), rough); return 0.5/max(den,1e-4); }

// PCG hash RNG + cosine-weighted hemisphere sampling for the path-traced GI integrator.
static inline uint pcg(thread uint& s){ s=s*747796405u+2891336453u; uint w=((s>>((s>>28u)+4u))^s)*277803737u; return (w>>22u)^w; }
static inline float rndf(thread uint& s){ return float(pcg(s))*(1.0/4294967296.0); }
static inline float3 cosineHemisphere(float3 n, float u1, float u2){
  float r=sqrt(u1), th=6.28318530718*u2;
  float3 t=normalize(cross((abs(n.x)>0.9?float3(0,1,0):float3(1,0,0)), n)); float3 b=cross(n,t);
  return normalize(t*(r*cos(th)) + b*(r*sin(th)) + n*sqrt(max(0.0,1.0-u1)));
}
static inline float gaussf(thread uint& s){ float u1=max(rndf(s),1e-6), u2=rndf(s); return sqrt(-2.0*log(u1))*cos(6.28318530718*u2); }
// Physical-camera post: linear exposure/gain, radial vignette, sRGB gamma, then gaussian sensor noise.
static inline float3 cameraPost(float3 lin, uint2 tid, constant Uniforms& u, thread uint& seed){
  lin *= (u.exposure>0.0 ? u.exposure : 1.0);
  if(u.vignette>0.0){ float2 p=(float2(float(tid.x),float(tid.y))+0.5)/float2(float(u.width),float(u.height))*2.0-1.0; lin *= max(0.0, 1.0 - u.vignette*dot(p,p)); }
  float3 o = pow(max(lin,0.0), 1.0/max(u.gamma,0.01));   // output gamma (OptiX camera.gamma; default 2.2)
  if(u.noiseSigma>0.0){ o += float3(gaussf(seed),gaussf(seed),gaussf(seed))*u.noiseSigma; }
  return clamp(o,0.0,1.0);
}

struct Hit { bool sky; float3 albedo; float3 n; float3 pos; uint mat; float dist; uint inst; float opacity; float rough; float metallic; float3 ks; float useSpec; float3 emissive; };

static Hit trace(ray r, instance_acceleration_structure accel,
   device const packed_float3* gN, device const packed_float3* gA, device const packed_float3* tint,
   device const float* iR, device const uint* nBase, device const uint* matI,
   device const float* gUV, device const int* gTexId, device const float* gOpacity, device const float* gRough, device const float* gMetallic, device const int* gRoughTexId, device const int* gMetalTexId, device const int* gOpacityTexId, device const packed_float3* gTangent, device const int* gNormalTexId, device const float* gSpecular, device const float* gEmissive, device const float* gTexScale, device const int* gKsTexId, device const int* gKeTexId, array<texture2d<float>,64> texs, sampler samp, texture2d<float> env, constant Uniforms& u){
  Hit h; h.sky=true; h.albedo=float3(0); h.n=float3(0,0,1); h.pos=r.origin; h.mat=0; h.dist=0; h.inst=0; h.opacity=1.0; h.rough=1.0; h.metallic=0.0; h.ks=float3(0); h.useSpec=0.0; h.emissive=float3(0);
  intersector<triangle_data,instancing> it; it.assume_geometry_type(geometry_type::triangle); it.force_opacity(forced_opacity::opaque);
  // OptiX (material_shaders.cu) uses the geometric world normal as-is -- it does NOT face-forward toward
  // the viewer. Face-forwarding made foliage-card shading view-dependent (trees lit only when zoomed out).
  float3 o0=r.origin;                                    // original origin (for true hit distance)
  for(int skip=0; skip<24; skip++){                      // loop to pass through alpha-cut-out (foliage) texels
    auto res=it.intersect(r,accel);
    if(res.type==intersection_type::none){ h.sky=true; h.albedo=skycol(r.direction,env,samp,u); return h; }
    uint id=res.instance_id, prim=res.primitive_id; float2 bc=res.triangle_barycentric_coord; float w0=1.0-bc.x-bc.y;
    uint tri=nBase[id]+prim;
    float3 on=normalize(w0*float3(gN[tri*3])+bc.x*float3(gN[tri*3+1])+bc.y*float3(gN[tri*3+2]));     // object normal
    float3 ot=normalize(w0*float3(gTangent[tri*3])+bc.x*float3(gTangent[tri*3+1])+bc.y*float3(gTangent[tri*3+2]));  // object tangent
    uint b=id*9u; float3 c0=float3(iR[b],iR[b+1],iR[b+2]),c1=float3(iR[b+3],iR[b+4],iR[b+5]),c2=float3(iR[b+6],iR[b+7],iR[b+8]);
    float3 hit=r.origin+res.distance*r.direction;
    uint mat=matI[id];
    float2 uv=w0*float2(gUV[tri*6],gUV[tri*6+1])+bc.x*float2(gUV[tri*6+2],gUV[tri*6+3])+bc.y*float2(gUV[tri*6+4],gUV[tri*6+5]); uv.y=1.0-uv.y;
    uv *= float2(gTexScale[tri*2],gTexScale[tri*2+1]);    // per-material UV scale (OptiX tex_scale)
    int texId=gTexId[tri]; float3 base;
    if(texId>=0){ float4 t=texs[texId].sample(samp,uv);
      if(t.a < 0.1){ r.origin=hit+r.direction*max(5e-3,length(hit)*3e-5); continue; }  // alpha cutout (coord-scaled advance)
      base=pow(t.rgb/max(t.a,1e-3),2.2); } else base=float3(gA[tri]);  // un-premultiply, then linearize sRGB tex (OptiX Pow 2.2)
    // Normal map (object space, matches OptiX material_shaders.cu): perturb the object normal, then transform to world.
    int ntx=gNormalTexId[tri];
    if(ntx>=0){ float3 bit=normalize(cross(on,ot)); float3 nd=texs[ntx].sample(samp,uv).rgb*2.0-1.0; on=normalize(nd.x*ot+nd.y*bit+nd.z*on); }
    float3 n=normalize(c0*on.x+c1*on.y+c2*on.z);          // world normal (after any normal-map perturbation)
    float3 albedo=base*float3(tint[id]);
    // Roughness/metallic come from PBR maps when present (OptiX samples map_Pr/map_Pm) -- this is what makes
    // the Audi paint glossy metallic; scalar GetRoughness()/GetMetallic() are only the fallback. Data maps
    // are linear (no sRGB), so sample .r directly.
    float rough=gRough[tri]; int rtx=gRoughTexId[tri]; if(rtx>=0) rough=texs[rtx].sample(samp,uv).r;
    float metal=gMetallic[tri]; int mtx=gMetalTexId[tri]; if(mtx>=0) metal=texs[mtx].sample(samp,uv).r;
    float opac=gOpacity[tri]; int otx=gOpacityTexId[tri]; if(otx>=0) opac=texs[otx].sample(samp,uv).r;  // map_d opacity (OptiX: opacity_tex overrides) -> glass/windows
    // Specular workflow (Ks/ks_tex, not sRGB-linearized -- matches OptiX legacy) + emissive (Ke*power, ke_tex)
    int ktx=gKsTexId[tri]; float3 ksv = ktx>=0 ? texs[ktx].sample(samp,uv).rgb : float3(gSpecular[tri*4],gSpecular[tri*4+1],gSpecular[tri*4+2]);
    int etx=gKeTexId[tri]; float3 kev = etx>=0 ? texs[etx].sample(samp,uv).rgb : float3(gEmissive[tri*4],gEmissive[tri*4+1],gEmissive[tri*4+2]);
    h.sky=false; h.dist=length(hit-o0); h.inst=id; h.albedo=albedo; h.n=n; h.pos=hit; h.mat=mat; h.opacity=opac; h.rough=rough; h.metallic=metal;
    h.ks=ksv; h.useSpec=gSpecular[tri*4+3]; h.emissive=kev*gEmissive[tri*4+3];
    return h;
  }
  h.sky=true; h.albedo=skycol(r.direction,env,samp,u); return h;
}

// Alpha-aware shadow ray: transparent (alpha-cutout) leaf/sign texels do NOT block light, exactly like
// Chrono's OptiX ShadowShader (shadow_shader.cuh: tex.w<1e-6 -> transparency 0 -> passes). Treating the
// foliage cards as opaque here caused dense canopy self-shadowing -> trees black at the top.
static float shadowRay(float3 origin, float3 dir, float maxd, float minD, instance_acceleration_structure accel,
   device const uint* nBase, device const int* gTexId, device const float* gUV, device const float* gOpacity, device const int* gOpacityTexId, device const float* gTexScale, array<texture2d<float>,64> texs, sampler samp){
  intersector<triangle_data,instancing> sit; sit.assume_geometry_type(geometry_type::triangle); sit.force_opacity(forced_opacity::opaque);
  float remaining = maxd;
  for(int i=0;i<16;i++){
    ray sr; sr.origin=origin; sr.direction=dir; sr.min_distance=minD; sr.max_distance=remaining;
    auto res=sit.intersect(sr,accel);
    if(res.type==intersection_type::none) return 1.0;        // reached the light unobstructed
    uint tri=nBase[res.instance_id]+res.primitive_id; int texId=gTexId[tri];
    float2 bc=res.triangle_barycentric_coord; float w0=1.0-bc.x-bc.y;
    float2 uv=w0*float2(gUV[tri*6],gUV[tri*6+1])+bc.x*float2(gUV[tri*6+2],gUV[tri*6+3])+bc.y*float2(gUV[tri*6+4],gUV[tri*6+5]);
    uv.y=1.0-uv.y; uv*=float2(gTexScale[tri*2],gTexScale[tri*2+1]);
    float opac=gOpacity[tri]; int otx=gOpacityTexId[tri]; if(otx>=0) opac=texs[otx].sample(samp,uv).r;
    if(opac < 0.5){ float adv=res.distance+minD; origin+=dir*adv; remaining-=adv; if(remaining<=minD) return 1.0; continue; }  // glass/transparent -> light passes (OptiX shadow uses opacity_tex)
    if(texId<0) return 0.0;                                   // untextured/opaque geometry -> shadowed
    if(texs[texId].sample(samp,uv).a >= 0.1) return 0.0;     // opaque texel -> shadowed
    float adv=res.distance+minD; origin+=dir*adv; remaining-=adv;  // transparent leaf -> pass through, continue
    if(remaining<=minD) return 1.0;
  }
  return 1.0;
}

// Direct-illumination shading, ported from Chrono's OptiX legacy shader (camera_legacy_shader.cuh):
// GGX Cook-Torrance specular + (1-F) Lambert diffuse per light, plus the OptiX "flashlight" ambient
// (a view-facing NdV term + an up-hemisphere term). No foliage/material special-casing -- the NdV
// ambient is exactly what keeps vertical tree cards from going dark under an overhead sun.
// Per-light direct illumination only (no ambient) -- used by both the legacy shader and the GI integrator.
static float3 directLighting(Hit h, float3 view, constant Uniforms& u, device const Light* lights, instance_acceleration_structure accel,
   device const uint* nBase, device const int* gTexId, device const float* gUV, device const float* gOpacity, device const int* gOpacityTexId, device const float* gTexScale, array<texture2d<float>,64> texs, sampler samp){
  float3 N = h.n;
  float NdV = max(dot(N, view), 0.0);
  float rough = clamp(h.rough, 0.045, 1.0);
  float metallic = clamp(h.metallic, 0.0, 1.0);
  // OptiX supports two workflows: specular (F0 = Ks*0.08) or metallic/roughness (default).
  bool specWF = h.useSpec > 0.5;
  float3 F0 = specWF ? (h.ks*0.08) : (metallic*h.albedo + (1.0-metallic)*float3(0.04));
  float3 diffAlbedo = specWF ? h.albedo : (h.albedo * (1.0-metallic));  // metals have no subsurface diffuse
  float3 col = float3(0.0);
  for(uint i=0;i<u.numLights;i++){
    Light L=lights[i]; float3 toL; float dL; float atten=1.0;
    if(L.type>1.5){ // SPOT: point-light falloff * angular cone falloff
      float3 d=float3(L.pos)-h.pos; dL=length(d); toL=d/max(dL,1e-4);
      if(L.range>0.0){ float as=0.01*L.range*L.range; atten=as/max(dL*dL,1e-4); }
      float cd=dot(-toL, normalize(float3(L.dir)));                       // cos angle from spot axis
      atten *= smoothstep(L.cosOuter, L.cosInner, cd);                    // inner=full, outer=0
    } else if(L.type>0.5){ toL=normalize(-float3(L.pos)); dL=1e4; }       // DIRECTIONAL
    else { float3 d=float3(L.pos)-h.pos; dL=length(d); toL=d/max(dL,1e-4); if(L.range>0.0){ float as=0.01*L.range*L.range; atten=as/max(dL*dL,1e-4); } }  // POINT: geom_term = atten_scale/dist^2, atten_scale=0.01*max_range^2
    float NdL = dot(N, toL);
    if(NdL <= 0.0) continue;                                     // light below the surface (OptiX: NdL<0 -> L=0)
    // Coordinate-scaled shadow epsilon: the NADS course spans ~1600 m, where a fixed 1e-3 offset is
    // below float precision -> shadow acne -> spurious self-shadowing -> "extremely dark". OptiX hides
    // this with EnableDynamicOrigin (recenters coords); we scale the epsilon with |hit| instead.
    float eps = max(5e-3, length(h.pos)*3e-5);
    float sh = shadowRay(h.pos+N*eps, toL, dL-eps, eps, accel, nBase, gTexId, gUV, gOpacity, gOpacityTexId, gTexScale, texs, samp);
    float3 incoming = float3(L.color) * (atten*sh) * NdL;        // ls.L * attenuation * NdL
    float3 hv = normalize(toL+view); float NdH=max(dot(N,hv),0.0), VdH=max(dot(view,hv),0.0);
    float3 F = F0 + (float3(1.0)-F0)*pow(1.0-VdH,5.0);           // Schlick Fresnel
    col += (float3(1.0)-F) * diffAlbedo * incoming;              // diffuse
    // Specular anti-aliasing: floor the roughness used for the *direct-light* specular lobe. At 1 spp the
    // razor-thin sun highlight on the car's sharp, high-curvature window-frame/panel edges is undersampled
    // and aliases into a flickering fringe (worst on the sun-facing side -- hence asymmetric). OptiX computes
    // the identical lobe and hides this with its AI denoiser; we instead widen the lobe just enough that it
    // can't be sharper than one sample resolves. Only affects the sun glint's sharpness; the sharp *mirror*
    // reflection (the "shiny" look) is untouched.
    float rspec = max(rough, 0.16);
    col += F * NormalDist(NdH,rspec) * HammonSmith(NdV,NdL,rspec) * incoming;  // Cook-Torrance specular
  }
  return col;
}

// Legacy shading = OptiX "flashlight" ambient (view-facing NdV + up-hemisphere) + direct lighting.
static float3 lighting(Hit h, float3 view, constant Uniforms& u, device const Light* lights, instance_acceleration_structure accel,
   device const uint* nBase, device const int* gTexId, device const float* gUV, device const float* gOpacity, device const int* gOpacityTexId, device const float* gTexScale, array<texture2d<float>,64> texs, sampler samp){
  float NdV = max(dot(h.n, view), 0.0);
  float3 amb = u.ambient * (NdV + (dot(h.n,float3(0,0,1))*0.5+0.5)) * h.albedo;
  return amb + directLighting(h,view,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp);
}

kernel void computeMain(uint2 tid [[thread_position_in_grid]], constant Uniforms& u [[buffer(0)]],
  device const packed_float3* gN [[buffer(1)]], device const packed_float3* gA [[buffer(2)]], device const float* iR [[buffer(3)]],
  instance_acceleration_structure accel [[buffer(4)]], device const uint* nBase [[buffer(5)]], device const uint* matI [[buffer(6)]],
  device const packed_float3* tint [[buffer(7)]], device const float* gUV [[buffer(8)]], device const int* gTexId [[buffer(9)]],
  device const Light* lights [[buffer(10)]], device const uint* instIds [[buffer(11)]], device const float* gOpacity [[buffer(12)]],
  device const float* gRough [[buffer(13)]], device const float* gMetallic [[buffer(14)]],
  device const int* gRoughTexId [[buffer(15)]], device const int* gMetalTexId [[buffer(16)]], device const int* gOpacityTexId [[buffer(17)]],
  device const packed_float3* gTangent [[buffer(18)]], device const int* gNormalTexId [[buffer(19)]],
  device const float* gSpecular [[buffer(20)]], device const float* gEmissive [[buffer(21)]], device const float* gTexScale [[buffer(22)]],
  device const int* gKsTexId [[buffer(23)]], device const int* gKeTexId [[buffer(24)]],
  array<texture2d<float>,64> texs [[texture(0)]], sampler samp [[sampler(0)]], texture2d<float, access::write> outTex [[texture(64)]], texture2d<float> envTex [[texture(65)]]) {
  if(tid.x>=u.width||tid.y>=u.height) return; float aspect=float(u.width)/float(u.height);

  // primary ray for the pixel center (used by all non-color modes)
  float ncx=(2.0*(float(tid.x)+0.5)/float(u.width)-1.0)*aspect;
  float ncy=(1.0-2.0*(float(tid.y)+0.5)/float(u.height));
  float3 cdir=camRayDir(ncx,ncy,u);

  if(u.mode==1u){ // DEPTH (distance to first hit; 0 = sky/miss)
    ray r; r.origin=float3(u.camPos); r.direction=cdir; r.min_distance=1e-3; r.max_distance=INFINITY;
    Hit h=trace(r,accel,gN,gA,tint,iR,nBase,matI,gUV,gTexId,gOpacity,gRough,gMetallic,gRoughTexId,gMetalTexId,gOpacityTexId,gTangent,gNormalTexId,gSpecular,gEmissive,gTexScale,gKsTexId,gKeTexId,texs,samp,envTex,u);
    outTex.write(float4(h.sky?0.0:h.dist,0,0,1), tid); return;
  }
  if(u.mode==2u){ // NORMAL (world-space; 0 = sky/miss)
    ray r; r.origin=float3(u.camPos); r.direction=cdir; r.min_distance=1e-3; r.max_distance=INFINITY;
    Hit h=trace(r,accel,gN,gA,tint,iR,nBase,matI,gUV,gTexId,gOpacity,gRough,gMetallic,gRoughTexId,gMetalTexId,gOpacityTexId,gTangent,gNormalTexId,gSpecular,gEmissive,gTexScale,gKsTexId,gKeTexId,texs,samp,envTex,u);
    outTex.write(h.sky?float4(0,0,0,1):float4(h.n,1.0), tid); return;
  }
  if(u.mode==3u){ // SEGMENTATION (class id, instance id)
    ray r; r.origin=float3(u.camPos); r.direction=cdir; r.min_distance=1e-3; r.max_distance=INFINITY;
    Hit h=trace(r,accel,gN,gA,tint,iR,nBase,matI,gUV,gTexId,gOpacity,gRough,gMetallic,gRoughTexId,gMetalTexId,gOpacityTexId,gTangent,gNormalTexId,gSpecular,gEmissive,gTexScale,gKsTexId,gKeTexId,texs,samp,envTex,u);
    float cls=0, inst=0; if(!h.sky){ cls=float(instIds[h.inst*2u]); inst=float(instIds[h.inst*2u+1u]); }
    outTex.write(float4(cls,inst,0,1), tid); return;
  }
  if(u.mode==4u){ // LIDAR: beam grid with sub-sampling (divergence) + return-mode reduction
    float3 fwd=float3(u.camForward), up=float3(u.camUp), leftv=-float3(u.camRight); // sensor +Y = -camRight
    float baseAz = -u.lidarHFov*0.5 + (float(tid.x)+0.5)/float(u.width) * u.lidarHFov;
    float baseEl = u.lidarVMin + ((u.height>1u)? float(tid.y)/float(u.height-1u) : 0.5) * (u.lidarVMax-u.lidarVMin);
    uint rad = max(u.lidarSampleRadius,1u); uint n = 2u*rad-1u;
    float firstR=1e9, firstI=0.0, lastR=0.0, sumR=0.0, sumI=0.0, strongR=0.0, strongI=-1.0; uint hits=0u;
    for(uint sj=0;sj<n;sj++) for(uint si=0;si<n;si++){
      float oaz=(n>1u)?((float(si)/float(n-1u))-0.5)*u.lidarHDiv:0.0;
      float oel=(n>1u)?((float(sj)/float(n-1u))-0.5)*u.lidarVDiv:0.0;
      float az=baseAz+oaz, el=baseEl+oel;
      float3 dir=normalize(cos(el)*(cos(az)*fwd + sin(az)*leftv) + sin(el)*up);
      ray r; r.origin=float3(u.camPos); r.direction=dir; r.min_distance=1e-3; r.max_distance=(u.maxDist>0.0)?u.maxDist:INFINITY;
      Hit h=trace(r,accel,gN,gA,tint,iR,nBase,matI,gUV,gTexId,gOpacity,gRough,gMetallic,gRoughTexId,gMetalTexId,gOpacityTexId,gTangent,gNormalTexId,gSpecular,gEmissive,gTexScale,gKsTexId,gKeTexId,texs,samp,envTex,u);
      if(!h.sky){ hits++;
        float inten=abs(dot(h.n,-dir));                                          // OptiX lidar: lidar_intensity(=1) * |N.V|
        sumR+=h.dist; sumI+=inten; lastR=max(lastR,h.dist);
        if(h.dist<firstR){ firstR=h.dist; firstI=inten; }
        if(inten>strongI){ strongI=inten; strongR=h.dist; } }
    }
    if(u.lidarReturnMode==4u){ // DUAL_RETURN: first + strongest, packed (firstR,firstI,strongR,strongI)
      outTex.write(hits>0u?float4(firstR,firstI,strongR,strongI):float4(0,0,0,0), tid); return;
    }
    float outR=0.0, outI=0.0;
    if(hits>0u){ uint rm=u.lidarReturnMode; float mI=sumI/float(hits);
      if(rm==2u){ outR=firstR; outI=mI; }        // FIRST_RETURN
      else if(rm==3u){ outR=lastR; outI=mI; }    // LAST_RETURN
      else if(rm==1u){ outR=sumR/float(hits); outI=mI; } // MEAN_RETURN
      else { outR=strongR; outI=strongI; }       // STRONGEST_RETURN
    }
    outTex.write(float4(outR,outI,0,1), tid); return;
  }

  if(u.mode==5u){ // RADAR: beam grid -> range, amplitude, hit instance index (Doppler resolved on host)
    float az = -u.lidarHFov*0.5 + (float(tid.x)+0.5)/float(u.width) * u.lidarHFov;
    float el = u.lidarVMin + ((u.height>1u)? float(tid.y)/float(u.height-1u) : 0.5) * (u.lidarVMax-u.lidarVMin);
    float3 fwd=float3(u.camForward), up=float3(u.camUp), leftv=-float3(u.camRight);
    float3 dir=normalize(cos(el)*(cos(az)*fwd + sin(az)*leftv) + sin(el)*up);
    ray r; r.origin=float3(u.camPos); r.direction=dir; r.min_distance=1e-3; r.max_distance=(u.maxDist>0.0)?u.maxDist:INFINITY;
    Hit h=trace(r,accel,gN,gA,tint,iR,nBase,matI,gUV,gTexId,gOpacity,gRough,gMetallic,gRoughTexId,gMetalTexId,gOpacityTexId,gTangent,gNormalTexId,gSpecular,gEmissive,gTexScale,gKsTexId,gKeTexId,texs,samp,envTex,u);
    if(h.sky){ outTex.write(float4(0,0,-1,0), tid); return; }
    float amp=abs(dot(h.n,-dir));   // OptiX radar: radar_backscatter(=1) * |N.V|
    outTex.write(float4(h.dist, amp, float(h.inst), 1.0), tid); return;  // b = hit instance index
  }

  // GI (mode 0 with use_gi): iterative path tracer -- NEE direct lighting per bounce + cosine-sampled
  // diffuse indirect bounces, Russian roulette, sky/env as the light. Ports camera_path_shader.cuh
  // (recursion unrolled to a loop). aa^2 = samples/pixel; opt-in (noisy at low spp, needs high spp or
  // a denoiser for a clean image), so the fast legacy path stays the default.
  if(u.useGi!=0u){
    uint spp=max(u.aa*u.aa,1u); uint seed=(tid.y*u.width+tid.x)*9781u+1u; float3 acc=float3(0.0);
    for(uint s=0;s<spp;s++){
      float jx=rndf(seed), jy=rndf(seed);
      float nx=(2.0*(float(tid.x)+jx)/float(u.width)-1.0)*aspect, ny=(1.0-2.0*(float(tid.y)+jy)/float(u.height));
      ray r; r.origin=float3(u.camPos); r.direction=camRayDir(nx,ny,u); r.min_distance=1e-3; r.max_distance=INFINITY;
      float3 thru=float3(1.0), rad=float3(0.0);
      for(int bnc=0;bnc<5;bnc++){
        Hit h=trace(r,accel,gN,gA,tint,iR,nBase,matI,gUV,gTexId,gOpacity,gRough,gMetallic,gRoughTexId,gMetalTexId,gOpacityTexId,gTangent,gNormalTexId,gSpecular,gEmissive,gTexScale,gKsTexId,gKeTexId,texs,samp,envTex,u);
        if(h.sky){ rad += thru*h.albedo; break; }                                  // sky/env is the light source
        float3 view=-r.direction;
        rad += thru * h.emissive * abs(dot(h.n,view));                             // emissive
        rad += thru * directLighting(h,view,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp);  // NEE direct lighting
        float3 nd=cosineHemisphere(h.n, rndf(seed), rndf(seed));                   // indirect diffuse bounce
        thru *= h.albedo * (1.0-clamp(h.metallic,0.0,1.0));                        // cosine estimator -> *albedo
        if(bnc>=2){ float p=clamp(max(max(thru.x,thru.y),thru.z),0.05,0.95); if(rndf(seed)>p) break; thru/=p; }  // Russian roulette
        float eps=max(5e-3,length(h.pos)*3e-5); r.origin=h.pos+h.n*eps; r.direction=nd; r.min_distance=1e-3; r.max_distance=INFINITY;
      }
      acc+=rad;   // (GI noise is handled by Russian roulette + the optional denoiser, like OptiX)
    }
    outTex.write(float4(cameraPost(acc/float(spp),tid,u,seed),1.0),tid); return;
  }

  // COLOR (mode 0): antialiased, lit, alpha-composited through transparent (glass) layers
  uint aa=max(u.aa,1u); float3 acc=float3(0.0); uint pseed=(tid.y*u.width+tid.x)*40503u+13u;
  for(uint sy=0;sy<aa;sy++) for(uint sx=0;sx<aa;sx++){
    float ox=(float(sx)+0.5)/float(aa), oy=(float(sy)+0.5)/float(aa);
    float nx=(2.0*(float(tid.x)+ox)/float(u.width)-1.0)*aspect;
    float ny=(1.0-2.0*(float(tid.y)+oy)/float(u.height));
    float3 dir=camRayDir(nx,ny,u);
    ray r; r.origin=float3(u.camPos); r.direction=dir; r.min_distance=1e-3; r.max_distance=INFINITY;
    if(u.apertureR>0.0){   // thin-lens depth of field: jitter the ray origin on the aperture, aim at the focal plane
      float3 fp=r.origin + dir*u.focalDist;
      float a1=rndf(pseed), a2=rndf(pseed), rad=u.apertureR*sqrt(a1), ang=6.28318530718*a2;
      r.origin += float3(u.camRight)*(rad*cos(ang)) + float3(u.camUp)*(rad*sin(ang));
      r.direction=normalize(fp - r.origin);
    }
    float3 color=float3(0.0); float trans=1.0; float primDist=-1.0;
    for(int layer=0; layer<8 && trans>0.02; layer++){
      Hit h=trace(r,accel,gN,gA,tint,iR,nBase,matI,gUV,gTexId,gOpacity,gRough,gMetallic,gRoughTexId,gMetalTexId,gOpacityTexId,gTangent,gNormalTexId,gSpecular,gEmissive,gTexScale,gKsTexId,gKeTexId,texs,samp,envTex,u);
      if(primDist<0.0 && !h.sky) primDist=h.dist;   // first surface distance (for fog)
      if(h.sky){ color += trans*h.albedo; trans=0.0; break; }
      float3 shaded = lighting(h,-r.direction,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp);
      shaded += h.emissive * abs(dot(h.n,-r.direction));   // emissive (OptiX: emissive_power*Ke*abs(NdV))
      if(h.opacity < 0.999){
        // Colored/semi-transparent surface (illum 9), exactly like OptiX legacy: the shaded surface color is
        // weighted by opacity and (1-opacity) passes straight through with no bend (CalculateRefractedColor).
        color += trans*h.opacity*shaded;      // opacity-weighted surface color (keeps tint)
        trans *= (1.0-h.opacity);             // remaining clear transmission
        r.origin = h.pos + r.direction*max(5e-3,length(h.pos)*3e-5);  // continue straight through (coord-scaled)
        continue;
      }
      // Mirror reflection -- ported verbatim from OptiX CameraLegacyShader::CalculateContributionToPixel.
      // The reflected color is weighted by the full Cook-Torrance BRDF (F*D*G*NdL/4pi) times the mirror
      // correction (1-rough)^2*metallic^2, CLAMPED to [0,1], and ADDED on top of the surface shading (not
      // lerped in). This BRDF weighting + clamp is exactly what keeps reflected high-frequency foliage from
      // aliasing into "crackle" speckle the way a raw mix() of the full-brightness reflection does.
      {
        float rr_rough = clamp(h.rough, 0.045, 1.0);
        float rr_metal = clamp(h.metallic, 0.0, 1.0);
        bool  rr_spec  = h.useSpec > 0.5;
        float3 rd = reflect(r.direction, h.n);
        float3 vv = -r.direction;
        float NdV = dot(h.n, vv);
        float NdL = dot(h.n, rd);
        float3 hw = normalize(rd + vv);          // halfway = normalize(next_dir - ray_dir)
        float NdH = dot(h.n, hw);
        float VdH = dot(vv, hw);
        float3 F;
        if(rr_spec){ float3 F0=h.ks*0.08; F = clamp(F0 + (float3(1.0)-F0)*pow(max(0.0,1.0-VdH),5.0), F0, float3(1.0)); }
        else       { F = rr_metal*h.albedo + (1.0-rr_metal)*float3(0.04); }
        float3 f_ct = F * NormalDist(NdH,rr_rough) * HammonSmith(NdV,NdL,rr_rough);
        float mirrorCorr = (1.0-h.rough)*(1.0-h.rough) * h.metallic*h.metallic;
        float3 w = clamp(mirrorCorr * f_ct * NdL / (4.0*3.14159265), 0.0, 1.0);
        if(dot(w, float3(0.30,0.59,0.11)) > 0.01){   // OptiX importance_cutoff
          // Single sharp mirror ray, exactly like OptiX legacy CalculateContributionToPixel. NB: on curved
          // low-roughness panels this reflects the car's own silhouette against the sky as a hard boundary
          // ("wavy" self-reflection); OptiX legacy has the same behaviour. (A glossy/blurred variant fixes
          // the look but costs many extra rays; see OPTIX_COMPARISON.md.)
          ray rr; rr.origin=h.pos+h.n*max(1e-2,length(h.pos)*3e-5); rr.direction=rd; rr.min_distance=1e-3; rr.max_distance=INFINITY;
          Hit hr=trace(rr,accel,gN,gA,tint,iR,nBase,matI,gUV,gTexId,gOpacity,gRough,gMetallic,gRoughTexId,gMetalTexId,gOpacityTexId,gTangent,gNormalTexId,gSpecular,gEmissive,gTexScale,gKsTexId,gKeTexId,texs,samp,envTex,u);
          float3 rc=hr.sky?hr.albedo:lighting(hr,-rd,u,lights,accel,nBase,gTexId,gUV,gOpacity,gOpacityTexId,gTexScale,texs,samp);
          shaded += w * rc;                          // ADDITIVE, BRDF-weighted (OptiX: color = mirror_reflection_color + ...)
        }
      }
      color += trans*shaded; trans=0.0; break;                   // opaque -> stop
    }
    color += trans*skycol(dir,envTex,samp,u);              // any remaining transmission shows the sky/background
    if(u.fogScatter>0.0 && primDist>0.0){ float ba=exp(-u.fogScatter*primDist); color=ba*color+(1.0-ba)*float3(u.fogColor); }  // exp fog (OptiX)
    acc+=color; }
  outTex.write(float4(cameraPost(acc/float(aa*aa),tid,u,pseed),1.0), tid);
}

// Portable despeckle/denoise pass (OptiX uses its AI denoiser; this is an edge-preserving spatial filter):
// clamp each pixel to its 3x3 neighbours' range to kill isolated speckle/fireflies, plus a light average
// blend to smooth residual noise. Removes the aa=1 edge shimmer on thin metallic frames + GI fireflies.
kernel void denoiseMain(uint2 tid [[thread_position_in_grid]],
   texture2d<float, access::read> inTex [[texture(0)]], texture2d<float, access::write> dstTex [[texture(1)]],
   constant uint2& dims [[buffer(0)]]) {
  if(tid.x>=dims.x||tid.y>=dims.y) return;
  float3 c=inTex.read(tid).rgb, mn=float3(1e9), mx=float3(-1e9), avg=c; int cnt=1;
  for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++){
    if(dx==0&&dy==0) continue;
    int2 p=clamp(int2(int(tid.x)+dx,int(tid.y)+dy), int2(0,0), int2(int(dims.x)-1,int(dims.y)-1));
    float3 s=inTex.read(uint2(p)).rgb; mn=min(mn,s); mx=max(mx,s); avg+=s; cnt++;
  }
  avg/=float(cnt);
  // Adaptive: smooth toward the local mean proportional to how much the neighbourhood varies. Flat regions
  // (small range) are left crisp; noisy/z-fighting bands & 1-spp aliasing (large range) get smoothed out.
  float3 rng=mx-mn; float r=max(rng.x,max(rng.y,rng.z));
  float k=clamp((r-0.05)*3.5, 0.0, 0.9);
  dstTex.write(float4(mix(c, avg, k),1.0), tid);
}
)MSLGEN";

#endif
