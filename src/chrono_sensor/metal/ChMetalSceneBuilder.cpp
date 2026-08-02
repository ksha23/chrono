// chrono_scene.cpp — Chrono ChSystem -> RenderScene. Backend-agnostic, cross-platform C++.
#include "chrono_sensor/metal/ChMetalSceneBuilder.h"
#include <fstream>
#include <cstdio>
#include <cmath>
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/assets/ChVisualShapeEllipsoid.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
using namespace chrono;
namespace cr {

struct V3 { double x,y,z; };
static V3 cross3(V3 a,V3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
static V3 sub3(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static double dot3(V3 a,V3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static V3 nrm3(V3 a){double l=std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z); if(l>0){a.x/=l;a.y/=l;a.z/=l;} return a;}
// orthonormalize tangent t against normal n (Gram-Schmidt); fall back to +X if degenerate
static V3 gramSchmidt(V3 t, V3 n){ double d=dot3(n,t); V3 r={t.x-n.x*d,t.y-n.y*d,t.z-n.z*d}; r=nrm3(r);
    if(r.x==0&&r.y==0&&r.z==0) r=V3{1,0,0}; return r; }

// tan9 (optional): 3 precomputed per-corner tangents (9 floats). If null, a flat per-face tangent is used.
static void addTri(Geometry& g, V3 a,V3 b,V3 c, V3 na,V3 nb,V3 nc, float* col, float* uvs, float* tan9=nullptr){
    g.verts.insert(g.verts.end(),{(float)a.x,(float)a.y,(float)a.z,(float)b.x,(float)b.y,(float)b.z,(float)c.x,(float)c.y,(float)c.z});
    g.normals.insert(g.normals.end(),{(float)na.x,(float)na.y,(float)na.z,(float)nb.x,(float)nb.y,(float)nb.z,(float)nc.x,(float)nc.y,(float)nc.z});
    g.colors.insert(g.colors.end(),{col[0],col[1],col[2]});
    if(uvs) g.uv.insert(g.uv.end(),{uvs[0],uvs[1],uvs[2],uvs[3],uvs[4],uvs[5]});
    else    g.uv.insert(g.uv.end(),{0,0,0,0,0,0});
    if(tan9){ g.tangents.insert(g.tangents.end(), tan9, tan9+9); return; }  // smooth per-vertex tangents (from meshToGeom)
    // fallback: flat per-face tangent from edges + UV deltas (primitives / no smoothing); default +X if degenerate
    V3 T{1,0,0};
    if(uvs){ V3 e1=sub3(b,a), e2=sub3(c,a);
        double du1=uvs[2]-uvs[0], dv1=uvs[3]-uvs[1], du2=uvs[4]-uvs[0], dv2=uvs[5]-uvs[1];
        double det=du1*dv2-du2*dv1;
        if(std::fabs(det)>1e-12){ double f=1.0/det; T=nrm3({f*(dv2*e1.x-dv1*e2.x), f*(dv2*e1.y-dv1*e2.y), f*(dv2*e1.z-dv1*e2.z)}); } }
    g.tangents.insert(g.tangents.end(),{(float)T.x,(float)T.y,(float)T.z,(float)T.x,(float)T.y,(float)T.z,(float)T.x,(float)T.y,(float)T.z});
}

// fill a Geometry (verts/normals/uv/colors) from a Chrono mesh; also emit per-face material index
static void meshToGeom(std::shared_ptr<ChTriangleMeshConnected> mesh, Geometry& g, std::vector<int>& faceMat,
                       const std::vector<std::shared_ptr<ChVisualMaterial>>& mats, float defR,float defG,float defB, ChVector3d scale){
    auto& Vs=mesh->GetCoordsVertices(); auto& Is=mesh->GetIndicesVertices(); size_t nv=Vs.size();
    auto& Ns=mesh->GetCoordsNormals(); auto& INs=mesh->GetIndicesNormals(); auto& MI=mesh->GetIndicesMaterials();
    auto& UVs=mesh->GetCoordsUV(); auto& IUV=mesh->GetIndicesUV();
    bool haveN=(!Ns.empty()&&INs.size()==Is.size()); bool haveMI=(MI.size()==Is.size()); bool haveM=!mats.empty();
    bool haveUV=(!UVs.empty()&&IUV.size()==Is.size());
    g={}; faceMat.clear();
    // Pass 1: accumulate smooth per-vertex tangents (averaged across faces) so normal mapping is smooth,
    // not faceted. Faceted per-face tangents made the car body look bumpy near the windows.
    std::vector<V3> accTan(nv, V3{0,0,0});
    if(haveUV){
        for(size_t ti=0;ti<Is.size();++ti){ auto&f=Is[ti]; int i0=f.x(),i1=f.y(),i2=f.z();
            if(i0<0||i1<0||i2<0||(size_t)i0>=nv||(size_t)i1>=nv||(size_t)i2>=nv) continue;
            auto&fu=IUV[ti]; size_t nu=UVs.size(); int u0=fu.x(),u1=fu.y(),u2=fu.z();
            if(!(u0>=0&&u1>=0&&u2>=0&&(size_t)u0<nu&&(size_t)u1<nu&&(size_t)u2<nu)) continue;
            V3 A{Vs[i0].x()*scale.x(),Vs[i0].y()*scale.y(),Vs[i0].z()*scale.z()};
            V3 B{Vs[i1].x()*scale.x(),Vs[i1].y()*scale.y(),Vs[i1].z()*scale.z()};
            V3 C{Vs[i2].x()*scale.x(),Vs[i2].y()*scale.y(),Vs[i2].z()*scale.z()};
            V3 e1=sub3(B,A), e2=sub3(C,A);
            double du1=UVs[u1].x()-UVs[u0].x(), dv1=UVs[u1].y()-UVs[u0].y(), du2=UVs[u2].x()-UVs[u0].x(), dv2=UVs[u2].y()-UVs[u0].y();
            double det=du1*dv2-du2*dv1; if(std::fabs(det)<1e-12) continue; double fdet=1.0/det;
            V3 T{fdet*(dv2*e1.x-dv1*e2.x), fdet*(dv2*e1.y-dv1*e2.y), fdet*(dv2*e1.z-dv1*e2.z)};
            for(int idx : {i0,i1,i2}){ accTan[idx].x+=T.x; accTan[idx].y+=T.y; accTan[idx].z+=T.z; }
        }
    }
    for(size_t ti=0;ti<Is.size();++ti){ auto&f=Is[ti]; int i0=f.x(),i1=f.y(),i2=f.z();
        if(i0<0||i1<0||i2<0||(size_t)i0>=nv||(size_t)i1>=nv||(size_t)i2>=nv) continue;
        V3 A{Vs[i0].x()*scale.x(),Vs[i0].y()*scale.y(),Vs[i0].z()*scale.z()};
        V3 B{Vs[i1].x()*scale.x(),Vs[i1].y()*scale.y(),Vs[i1].z()*scale.z()};
        V3 C{Vs[i2].x()*scale.x(),Vs[i2].y()*scale.y(),Vs[i2].z()*scale.z()};
        V3 nA,nB,nC;
        if(haveN){ auto&fn=INs[ti]; size_t nn=Ns.size(); int j0=fn.x(),j1=fn.y(),j2=fn.z();
            if(j0>=0&&j1>=0&&j2>=0&&(size_t)j0<nn&&(size_t)j1<nn&&(size_t)j2<nn){ ChVector3d is(1.0/scale.x(),1.0/scale.y(),1.0/scale.z());  // inverse-transpose for non-uniform scale
                nA=nrm3({Ns[j0].x()*is.x(),Ns[j0].y()*is.y(),Ns[j0].z()*is.z()}); nB=nrm3({Ns[j1].x()*is.x(),Ns[j1].y()*is.y(),Ns[j1].z()*is.z()}); nC=nrm3({Ns[j2].x()*is.x(),Ns[j2].y()*is.y(),Ns[j2].z()*is.z()});}
            else { nA=nrm3(cross3(sub3(B,A),sub3(C,A))); nB=nA;nC=nA; } }
        else { nA=nrm3(cross3(sub3(B,A),sub3(C,A))); nB=nA;nC=nA; }
        int mi = haveMI ? MI[ti] : 0;
        float col[3]={defR,defG,defB};
        if(haveM && mi>=0 && mi<(int)mats.size()){ auto kd=mats[mi]->GetDiffuseColor(); col[0]=kd.R;col[1]=kd.G;col[2]=kd.B; }
        float uvs[6]={0,0,0,0,0,0};
        if(haveUV){ auto&fu=IUV[ti]; size_t nu=UVs.size(); int u0=fu.x(),u1=fu.y(),u2=fu.z();
            if(u0>=0&&u1>=0&&u2>=0&&(size_t)u0<nu&&(size_t)u1<nu&&(size_t)u2<nu){ uvs[0]=UVs[u0].x();uvs[1]=UVs[u0].y();uvs[2]=UVs[u1].x();uvs[3]=UVs[u1].y();uvs[4]=UVs[u2].x();uvs[5]=UVs[u2].y(); } }
        float tan9[9]; float* tptr=nullptr;
        if(haveUV){ V3 g0=gramSchmidt(accTan[i0],nA), g1=gramSchmidt(accTan[i1],nB), g2=gramSchmidt(accTan[i2],nC);
            tan9[0]=g0.x;tan9[1]=g0.y;tan9[2]=g0.z; tan9[3]=g1.x;tan9[4]=g1.y;tan9[5]=g1.z; tan9[6]=g2.x;tan9[7]=g2.y;tan9[8]=g2.z; tptr=tan9; }
        addTri(g,A,B,C,nA,nB,nC,col,uvs,tptr);
        faceMat.push_back(haveM?mi:-1);
    }
}
// primitive generators (object space; white base color; no UV)
static void genBox(double lx,double ly,double lz,Geometry& g){ g={}; double x=lx*.5,y=ly*.5,z=lz*.5; float w[3]={1,1,1};
    auto q=[&](V3 a,V3 b,V3 c,V3 d,V3 nn){ addTri(g,a,b,c,nn,nn,nn,w,nullptr); addTri(g,a,c,d,nn,nn,nn,w,nullptr); };
    q({-x,-y,z},{x,-y,z},{x,y,z},{-x,y,z},{0,0,1}); q({x,-y,-z},{-x,-y,-z},{-x,y,-z},{x,y,-z},{0,0,-1});
    q({-x,-y,-z},{x,-y,-z},{x,-y,z},{-x,-y,z},{0,-1,0}); q({x,y,-z},{-x,y,-z},{-x,y,z},{x,y,z},{0,1,0});
    q({x,-y,-z},{x,y,-z},{x,y,z},{x,-y,z},{1,0,0}); q({-x,y,-z},{-x,-y,-z},{-x,-y,z},{-x,y,z},{-1,0,0}); }
static void genSphere(double sx,double sy,double sz,int st,int sl,Geometry& g){ g={}; float w[3]={1,1,1};
    auto P=[&](int i,int j){ double th=M_PI*i/st,ph=2*M_PI*j/sl; return V3{sin(th)*cos(ph),sin(th)*sin(ph),cos(th)}; };
    auto vN=[&](V3 d){ return nrm3(V3{d.x/sx,d.y/sy,d.z/sz}); };
    auto sc=[&](V3 d){ return V3{d.x*sx,d.y*sy,d.z*sz}; };
    for(int i=0;i<st;i++)for(int j=0;j<sl;j++){ V3 a=P(i,j),b=P(i,j+1),c=P(i+1,j),d=P(i+1,j+1);
        addTri(g,sc(a),sc(b),sc(c),vN(a),vN(b),vN(c),w,nullptr); addTri(g,sc(b),sc(d),sc(c),vN(b),vN(d),vN(c),w,nullptr); } }
static void genCyl(double r,double h,int seg,Geometry& g){ g={}; double hz=h*.5; float w[3]={1,1,1};
    for(int i=0;i<seg;i++){ double a0=2*M_PI*i/seg,a1=2*M_PI*(i+1)/seg; V3 d0{cos(a0),sin(a0),0},d1{cos(a1),sin(a1),0};
        V3 p00{d0.x*r,d0.y*r,-hz},p01{d1.x*r,d1.y*r,-hz},p10{d0.x*r,d0.y*r,hz},p11{d1.x*r,d1.y*r,hz};
        addTri(g,p00,p01,p10,d0,d1,d0,w,nullptr); addTri(g,p01,p11,p10,d1,d1,d0,w,nullptr);
        addTri(g,{0,0,hz},p10,p11,{0,0,1},{0,0,1},{0,0,1},w,nullptr); addTri(g,{0,0,-hz},p01,p00,{0,0,-1},{0,0,-1},{0,0,-1},w,nullptr); } }
static void genCone(double r,double h,int seg,Geometry& g){ g={}; double hz=h*.5; float w[3]={1,1,1}; V3 ap{0,0,hz};
    for(int i=0;i<seg;i++){ double a0=2*M_PI*i/seg,a1=2*M_PI*(i+1)/seg; V3 b0{r*cos(a0),r*sin(a0),-hz},b1{r*cos(a1),r*sin(a1),-hz};
        V3 n0=nrm3({cos(a0),sin(a0),r/h}),n1=nrm3({cos(a1),sin(a1),r/h});
        addTri(g,b0,b1,ap,n0,n1,nrm3({(n0.x+n1.x)/2,(n0.y+n1.y)/2,(n0.z+n1.z)/2}),w,nullptr);
        addTri(g,{0,0,-hz},b1,b0,{0,0,-1},{0,0,-1},{0,0,-1},w,nullptr); } }
static void genCapsule(double r,double h,int seg,Geometry& g){ genCyl(r,h,seg,g); double hz=h*.5; float w[3]={1,1,1}; int rings=6;
    auto hemi=[&](double zc,double sgn){ for(int i=0;i<rings;i++)for(int j=0;j<seg;j++){
        double t0=(M_PI/2)*i/rings,t1=(M_PI/2)*(i+1)/rings,p0=2*M_PI*j/seg,p1=2*M_PI*(j+1)/seg;
        auto pt=[&](double t,double p){ V3 d{sin(t)*cos(p),sin(t)*sin(p),sgn*cos(t)}; return std::pair<V3,V3>({d.x*r,d.y*r,zc+d.z*r},d); };
        auto A=pt(t0,p0),B=pt(t0,p1),C=pt(t1,p0),D=pt(t1,p1);
        addTri(g,A.first,B.first,C.first,A.second,B.second,C.second,w,nullptr); addTri(g,B.first,D.first,C.first,B.second,D.second,C.second,w,nullptr); } };
    hemi(hz,1.0); hemi(-hz,-1.0); }

int ChScene::countShapes() const {
    int c=0;
    for(auto& it: sys_->GetOtherPhysicsItems()){ auto vm=it->GetVisualModel(); if(vm) c+=(int)vm->GetShapeInstances().size(); }
    for(auto& b: sys_->GetBodies()){ auto vm=b->GetVisualModel(); if(vm) c+=(int)vm->GetShapeInstances().size(); }
    return c;
}
bool ChScene::topologyChanged() const { return countShapes()!=lastShapeCount_; }

static void fillFrame(Instance& in, const ChFramed& F){
    auto R=F.GetRotMat(); ChVector3d p=F.GetPos();
    in.xform[0]=R(0,0);in.xform[1]=R(1,0);in.xform[2]=R(2,0); in.xform[3]=R(0,1);in.xform[4]=R(1,1);in.xform[5]=R(2,1);
    in.xform[6]=R(0,2);in.xform[7]=R(1,2);in.xform[8]=R(2,2); in.xform[9]=p.x();in.xform[10]=p.y();in.xform[11]=p.z();
    for(int k=0;k<9;k++) in.rot[k]=in.xform[k];
}

void ChScene::build(RenderScene& scene){
    scene.geometries.clear(); scene.instances.clear(); scene.texturePaths.clear();
    srcs_.clear(); geomCache_.clear();
    std::map<std::string,int> texIdx;
    std::string curBaseDir;              // directory of the current mesh file (resolve relative map_Kd)
    uint32_t curClass=0, curInst=0;      // semantic class/instance ids of the current shape
    auto fileOk=[](const std::string& p){ std::ifstream f(p); return f.good(); };
    auto texturePathIndex=[&](const std::string& raw)->int{
        if(raw.empty()) return -1; std::string p=raw;
        if(!fileOk(p)){
            if(!curBaseDir.empty() && fileOk(curBaseDir+"/"+raw)) p=curBaseDir+"/"+raw;      // relative to the mesh's own dir
            else if(fileOk(GetChronoDataPath()+raw)) p=GetChronoDataPath()+raw;              // relative to the Chrono data path
        }
        auto it=texIdx.find(p); if(it!=texIdx.end()) return it->second;
        if((int)scene.texturePaths.size()>=64) return -1;   // shader texs[] array is fixed at 64; refuse beyond it (avoids OOB reads)
        int idx=(int)scene.texturePaths.size(); scene.texturePaths.push_back(p); texIdx[p]=idx; return idx;
    };
    std::vector<int> faceMat;
    auto toTexIds=[&](Geometry& g, std::vector<int>& fm, const std::vector<std::shared_ptr<ChVisualMaterial>>& mats){
        g.texId.clear(); g.texId.reserve(fm.size()); g.opacity.clear(); g.opacity.reserve(fm.size());
        g.roughness.clear(); g.roughness.reserve(fm.size()); g.metallic.clear(); g.metallic.reserve(fm.size());
        g.roughTexId.clear(); g.roughTexId.reserve(fm.size()); g.metalTexId.clear(); g.metalTexId.reserve(fm.size());
        g.opacityTexId.clear(); g.opacityTexId.reserve(fm.size()); g.normalTexId.clear(); g.normalTexId.reserve(fm.size());
        g.specular.clear(); g.emissive.clear(); g.texScale.clear(); g.ksTexId.clear(); g.keTexId.clear();
        for(int mi:fm){ int t=-1, rtx=-1, mtx=-1, otx=-1, ntx=-1, ktx=-1, etx=-1; float op=1.f, rg=1.f, mt=0.f;
            float ks[3]={0,0,0}, usp=0.f, ke[3]={0,0,0}, ep=0.f, ts[2]={1.f,1.f};
            if(mi>=0&&mi<(int)mats.size()){ auto& M=*mats[mi]; t=texturePathIndex(M.GetKdTexture()); op=M.GetOpacity(); rg=M.GetRoughness(); mt=M.GetMetallic();
                rtx=texturePathIndex(M.GetRoughnessTexture()); mtx=texturePathIndex(M.GetMetallicTexture()); otx=texturePathIndex(M.GetOpacityTexture());
                ntx=texturePathIndex(M.GetNormalMapTexture()); ktx=texturePathIndex(M.GetKsTexture()); etx=texturePathIndex(M.GetKeTexture());
                auto ksc=M.GetSpecularColor(); ks[0]=ksc.R; ks[1]=ksc.G; ks[2]=ksc.B; usp=M.GetUseSpecularWorkflow()?1.f:0.f;
                auto kec=M.GetEmissiveColor(); ke[0]=kec.R; ke[1]=kec.G; ke[2]=kec.B; ep=M.GetEmissivePower();
                auto tsc=M.GetTextureScale(); ts[0]=tsc.x(); ts[1]=tsc.y(); }
            g.texId.push_back(t); g.opacity.push_back(op); g.roughness.push_back(rg); g.metallic.push_back(mt);
            g.roughTexId.push_back(rtx); g.metalTexId.push_back(mtx); g.opacityTexId.push_back(otx); g.normalTexId.push_back(ntx);
            g.specular.insert(g.specular.end(),{ks[0],ks[1],ks[2],usp}); g.emissive.insert(g.emissive.end(),{ke[0],ke[1],ke[2],ep});
            g.texScale.insert(g.texScale.end(),{ts[0],ts[1]}); g.ksTexId.push_back(ktx); g.keTexId.push_back(etx); }
    };
    auto shapeColor=[&](std::shared_ptr<ChVisualShape> sh,float* out){ auto& m=sh->GetMaterials(); if(!m.empty()){auto kd=m[0]->GetDiffuseColor(); out[0]=kd.R;out[1]=kd.G;out[2]=kd.B;} };

    auto storeSF=[&](InstSrc& s, const ChFramed& f){ auto q=f.GetRot(); auto p=f.GetPos(); s.sf[0]=q.e0();s.sf[1]=q.e1();s.sf[2]=q.e2();s.sf[3]=q.e3(); s.sf[4]=p.x();s.sf[5]=p.y();s.sf[6]=p.z(); };

    auto addInstance=[&](int geom, ChBody* body, const ChFramed& sf, float* tint, uint32_t mat, bool dynamic, std::shared_ptr<ChTriangleMeshConnected> dynMesh){
        Instance in; in.geom=geom; in.mat=mat; if(tint){in.tint[0]=tint[0];in.tint[1]=tint[1];in.tint[2]=tint[2];}
        in.classId=curClass; in.instanceId=curInst;
        scene.instances.push_back(in);
        InstSrc s; s.body=body; s.geom=geom; s.dynamic=dynamic; s.mesh=dynMesh; storeSF(s,sf); srcs_.push_back(s);
    };

    auto processItem=[&](ChObj* obj, ChBody* body, bool isTerrain){
        auto vm=obj->GetVisualModel(); if(!vm) return;
        std::string nm=obj->GetName(); float vdef[3]={0.55f,0.55f,0.58f};
        if(nm.find("heel")!=std::string::npos||nm.find("ire")!=std::string::npos||nm.find("pindle")!=std::string::npos){vdef[0]=0.06f;vdef[1]=0.06f;vdef[2]=0.07f;}
        for(auto& si: vm->GetShapeInstances()){
            auto sh=si.shape; ChFramed sf=si.frame; Geometry g;
            { auto& mm=sh->GetMaterials(); curClass = mm.empty()?0u:(uint32_t)mm[0]->GetClassID(); curInst = mm.empty()?0u:(uint32_t)mm[0]->GetInstanceID(); }
            curBaseDir.clear();
            if(auto tm=std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(sh)){
                auto mesh=tm->GetMesh(); if(!mesh) continue;
                if(isTerrain){ meshToGeom(mesh,g,faceMat,{},0.52f,0.40f,0.27f,ChVector3d(1,1,1)); if(g.verts.empty()) continue; g.texId.assign(g.triCount(),-1); g.opacity.assign(g.triCount(),1.0f); g.roughness.assign(g.triCount(),1.0f); g.metallic.assign(g.triCount(),0.0f); g.roughTexId.assign(g.triCount(),-1); g.metalTexId.assign(g.triCount(),-1); g.opacityTexId.assign(g.triCount(),-1); g.normalTexId.assign(g.triCount(),-1); g.specular.assign(g.triCount()*4,0.f); g.emissive.assign(g.triCount()*4,0.f); g.texScale.assign(g.triCount()*2,1.f); g.ksTexId.assign(g.triCount(),-1); g.keTexId.assign(g.triCount(),-1); g.dynamic=true;
                    int gi=(int)scene.geometries.size(); scene.geometries.push_back(std::move(g)); float t[3]={1,1,1}; addInstance(gi,nullptr,sf,t,2,true,mesh); }
                else { char k[64]; snprintf(k,64,"mesh:%p",(void*)mesh.get());
                    if(!geomCache_.count(k)){ meshToGeom(mesh,g,faceMat,tm->GetMaterials(),vdef[0],vdef[1],vdef[2],ChVector3d(1,1,1)); if(g.verts.empty()) continue; toTexIds(g,faceMat,tm->GetMaterials()); geomCache_[k]=(int)scene.geometries.size(); scene.geometries.push_back(std::move(g)); }
                    float t[3]={1,1,1}; addInstance(geomCache_[k],body,sf,t,0,false,nullptr); }
            } else if(auto mf=std::dynamic_pointer_cast<ChVisualShapeModelFile>(sh)){
                std::string fn=mf->GetFilename(); ChVector3d sc=mf->GetScale(); char k[600]; snprintf(k,600,"file:%s:%g,%g,%g",fn.c_str(),sc.x(),sc.y(),sc.z());
                { auto sl=fn.find_last_of("/\\"); curBaseDir = (sl==std::string::npos)?std::string():fn.substr(0,sl); }
                if(!geomCache_.count(k)){ auto mesh=ChTriangleMeshConnected::CreateFromWavefrontFile(fn,true,true); if(!mesh) continue;
                    float col[3]={vdef[0],vdef[1],vdef[2]}; shapeColor(sh,col); meshToGeom(mesh,g,faceMat,mf->GetMaterials(),col[0],col[1],col[2],sc); if(g.verts.empty()) continue; toTexIds(g,faceMat,mf->GetMaterials()); geomCache_[k]=(int)scene.geometries.size(); scene.geometries.push_back(std::move(g)); }
                float t[3]={1,1,1}; addInstance(geomCache_[k],body,sf,t,0,false,nullptr);
            } else {
                char k[96]={0}; bool ok=true; float col[3]={vdef[0],vdef[1],vdef[2]}; shapeColor(sh,col);
                if(auto s2=std::dynamic_pointer_cast<ChVisualShapeBox>(sh)){ auto L=s2->GetLengths(); snprintf(k,96,"box:%.4f,%.4f,%.4f",L.x(),L.y(),L.z()); if(!geomCache_.count(k)){genBox(L.x(),L.y(),L.z(),g);} }
                else if(auto s2=std::dynamic_pointer_cast<ChVisualShapeSphere>(sh)){ double r=s2->GetRadius(); snprintf(k,96,"sph:%.4f",r); if(!geomCache_.count(k)){genSphere(r,r,r,16,28,g);} }
                else if(auto s2=std::dynamic_pointer_cast<ChVisualShapeCylinder>(sh)){ double r=s2->GetRadius(),h=s2->GetHeight(); snprintf(k,96,"cyl:%.4f,%.4f",r,h); if(!geomCache_.count(k)){genCyl(r,h,24,g);} }
                else if(auto s2=std::dynamic_pointer_cast<ChVisualShapeCapsule>(sh)){ double r=s2->GetRadius(),h=s2->GetHeight(); snprintf(k,96,"cap:%.4f,%.4f",r,h); if(!geomCache_.count(k)){genCapsule(r,h,24,g);} }
                else if(auto s2=std::dynamic_pointer_cast<ChVisualShapeCone>(sh)){ double r=s2->GetRadius(),h=s2->GetHeight(); snprintf(k,96,"cone:%.4f,%.4f",r,h); if(!geomCache_.count(k)){genCone(r,h,24,g);} }
                else if(auto s2=std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(sh)){ auto S=s2->GetSemiaxes(); snprintf(k,96,"ell:%.4f,%.4f,%.4f",S.x(),S.y(),S.z()); if(!geomCache_.count(k)){genSphere(S.x(),S.y(),S.z(),16,28,g);} }
                else ok=false;
                if(!ok) continue;
                if(!geomCache_.count(k)){ g.texId.assign(g.triCount(),-1); g.opacity.assign(g.triCount(),1.0f); g.roughness.assign(g.triCount(),1.0f); g.metallic.assign(g.triCount(),0.0f); g.roughTexId.assign(g.triCount(),-1); g.metalTexId.assign(g.triCount(),-1); g.opacityTexId.assign(g.triCount(),-1); g.normalTexId.assign(g.triCount(),-1); g.specular.assign(g.triCount()*4,0.f); g.emissive.assign(g.triCount()*4,0.f); g.texScale.assign(g.triCount()*2,1.f); g.ksTexId.assign(g.triCount(),-1); g.keTexId.assign(g.triCount(),-1); geomCache_[k]=(int)scene.geometries.size(); scene.geometries.push_back(std::move(g)); }
                addInstance(geomCache_[k],body,sf,col,0,false,nullptr);
            }
        }
    };
    for(auto& it: sys_->GetOtherPhysicsItems()) processItem(it.get(),nullptr,true);
    for(auto& b: sys_->GetBodies()) processItem(b.get(),b.get(),false);
    if(groundOn_){ Geometry g={}; double gg=groundSize_,z=groundZ_; float w[3]={0.5f,0.5f,0.52f};
        addTri(g,{-gg,-gg,z},{-gg,gg,z},{gg,gg,z},{0,0,1},{0,0,1},{0,0,1},w,nullptr); addTri(g,{-gg,-gg,z},{gg,gg,z},{gg,-gg,z},{0,0,1},{0,0,1},{0,0,1},w,nullptr);
        g.texId.assign(2,-1); g.opacity.assign(2,1.0f); g.roughness.assign(2,1.0f); g.metallic.assign(2,0.0f); g.roughTexId.assign(2,-1); g.metalTexId.assign(2,-1); g.opacityTexId.assign(2,-1); g.normalTexId.assign(2,-1); g.specular.assign(8,0.f); g.emissive.assign(8,0.f); g.texScale.assign(4,1.f); g.ksTexId.assign(2,-1); g.keTexId.assign(2,-1); int gi=(int)scene.geometries.size(); scene.geometries.push_back(std::move(g));
        float t[3]={1,1,1}; addInstance(gi,nullptr,ChFramed(),t,groundChecker_?1u:0u,false,nullptr); }

    lastShapeCount_=countShapes();
    refresh(scene);
}

void ChScene::refresh(RenderScene& scene){
    std::vector<int> fmTmp; Geometry gTmp;
    for(size_t i=0;i<srcs_.size() && i<scene.instances.size(); ++i){
        InstSrc& s=srcs_[i]; Instance& in=scene.instances[i];
        ChFramed shapeF(ChVector3d(s.sf[4],s.sf[5],s.sf[6]), ChQuaterniond(s.sf[0],s.sf[1],s.sf[2],s.sf[3]));
        // Compose against the body's VISUAL-MODEL frame, not GetPos()/GetRot(): for a
        // ChBodyAuxRef (e.g. a vehicle chassis) visuals attach at the reference frame, while
        // GetPos() is the center of mass. Using GetPos() renders the body offset by the COM
        // gap (the Audi COM is 0.55 m above its ref -> body floats above the wheels).
        ChFramed world = s.body ? (shapeF >> s.body->GetVisualModelFrame()) : shapeF;
        fillFrame(in, world);
        if(s.body){ auto v=s.body->GetPosDt(); in.vel[0]=(float)v.x(); in.vel[1]=(float)v.y(); in.vel[2]=(float)v.z(); }
        else { in.vel[0]=in.vel[1]=in.vel[2]=0.f; }
        if(s.dynamic && s.mesh){                       // re-extract deforming mesh in place
            Geometry& g=scene.geometries[s.geom];
            meshToGeom(s.mesh,gTmp,fmTmp,{},0.52f,0.40f,0.27f,ChVector3d(1,1,1));
            if(gTmp.triCount()==g.triCount()){ g.verts.swap(gTmp.verts); g.normals.swap(gTmp.normals); }
        }
    }
}

}  // namespace cr
